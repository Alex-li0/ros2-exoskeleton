#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;

class SingleLegCPGNode : public rclcpp::Node
{
public:
  SingleLegCPGNode() : Node("single_leg_cpg_node")
  {
    // 1. 声明 CPG 底层数学参数
    this->declare_parameter("mu", 1.0);
    this->declare_parameter("omega", 6.2831853);
    this->declare_parameter("gamma", 10.0);
    
    // 2. 声明 11 个由 OLS 精确求解出的映射权重参数
    // 髋关节 (Hip)
    this->declare_parameter("w_hx", 0.0);
    this->declare_parameter("w_hy", 0.4);
    this->declare_parameter("h_offset", 0.1);
    
    // 膝关节 (Knee) - 包含基频和二倍频权重
    this->declare_parameter("w_kx1", 0.0);
    this->declare_parameter("w_ky1", 0.5);
    this->declare_parameter("w_kx2", 0.0);
    this->declare_parameter("w_ky2", 0.2);
    this->declare_parameter("k_offset", 0.5);
    
    // 踝关节 (Ankle)
    this->declare_parameter("w_ax", 0.0);
    this->declare_parameter("w_ay", 0.15);
    this->declare_parameter("a_offset", -0.05);

    // 3. 从参数服务器拉取初始值并绑定动态修改回调
    update_parameters_from_server();
    param_subscriber_ = this->add_on_set_parameters_callback(
      std::bind(&SingleLegCPGNode::parametersCallback, this, std::placeholders::_1));
    
    // 4. 初始化发布者和节点状态
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    
    // 确保这里的关节名字和你的 URDF 完全对应
    joint_names_ = {"hip1", "hip2", "hip3", "knee", "ankle", "camera"};
    
    // CPG 核心状态机只需 x 和 y 两个变量 (心跳主振荡器)
    x_ = 0.1; 
    y_ = 0.1;
    dt_ctrl_ = 0.01; // ROS 节点的控制发送周期 10ms (100Hz)

    // 创建 100Hz 的定时器
    timer_ = this->create_wall_timer(
      10ms, std::bind(&SingleLegCPGNode::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "CPG 控制器已启动");
  }

private:
  void update_parameters_from_server()
  {
    mu_ = this->get_parameter("mu").as_double();
    omega_ = this->get_parameter("omega").as_double();
    gamma_ = this->get_parameter("gamma").as_double();
    
    w_hx_ = this->get_parameter("w_hx").as_double();
    w_hy_ = this->get_parameter("w_hy").as_double();
    h_off_ = this->get_parameter("h_offset").as_double();
    
    w_kx1_ = this->get_parameter("w_kx1").as_double();
    w_ky1_ = this->get_parameter("w_ky1").as_double();
    w_kx2_ = this->get_parameter("w_kx2").as_double();
    w_ky2_ = this->get_parameter("w_ky2").as_double();
    k_off_ = this->get_parameter("k_offset").as_double();
    
    w_ax_ = this->get_parameter("w_ax").as_double();
    w_ay_ = this->get_parameter("w_ay").as_double();
    a_off_ = this->get_parameter("a_offset").as_double();
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters) {
      if (param.get_name() == "mu") { mu_ = param.as_double(); }
      else if (param.get_name() == "omega") { omega_ = param.as_double(); }
      else if (param.get_name() == "gamma") { gamma_ = param.as_double(); }
      
      else if (param.get_name() == "w_hx") { w_hx_ = param.as_double(); }
      else if (param.get_name() == "w_hy") { w_hy_ = param.as_double(); }
      else if (param.get_name() == "h_offset") { h_off_ = param.as_double(); }
      
      else if (param.get_name() == "w_kx1") { w_kx1_ = param.as_double(); }
      else if (param.get_name() == "w_ky1") { w_ky1_ = param.as_double(); }
      else if (param.get_name() == "w_kx2") { w_kx2_ = param.as_double(); }
      else if (param.get_name() == "w_ky2") { w_ky2_ = param.as_double(); }
      else if (param.get_name() == "k_offset") { k_off_ = param.as_double(); }
      
      else if (param.get_name() == "w_ax") { w_ax_ = param.as_double(); }
      else if (param.get_name() == "w_ay") { w_ay_ = param.as_double(); }
      else if (param.get_name() == "a_offset") { a_off_ = param.as_double(); }
    }
    return result;
  }

  void timer_callback()
  {
    // ========================================================
    // 1. 核心数学层：使用子步长 (Sub-stepping) 求解欧拉微分方程
    // 彻底消除由于 100Hz 频率过低导致的数值积分毛刺
    // ========================================================
    int sub_steps = 10; 
    double dt_sim = dt_ctrl_ / sub_steps; // 内部积分步长为 1ms

    for(int i = 0; i < sub_steps; i++) {
        double r2 = x_ * x_ + y_ * y_;
        double dx = gamma_ * (mu_ - r2) * x_ - omega_ * y_;
        double dy = gamma_ * (mu_ - r2) * y_ + omega_ * x_;
        
        x_ += dx * dt_sim;
        y_ += dy * dt_sim;
    }

    // ========================================================
    // 2. 模式形成层：利用 OLS 权重将纯净的 x,y 波形映射为物理关节
    // ========================================================
    double target_hip1 = 0.0;
    double target_hip2 = 0.0;
    
    // 髋关节：线性组合
    double target_hip3 = w_hx_ * x_ + w_hy_ * y_ + h_off_;       
    
    // 膝关节：基频 + 利用二倍角公式无损生成的倍频特征 (塑造双峰)
    double x2_minus_y2 = x_ * x_ - y_ * y_; // 完美等价于 cos(2wt)
    double two_x_y = 2.0 * x_ * y_;         // 完美等价于 sin(2wt)
    double target_knee = w_kx1_ * x_ + w_ky1_ * y_ + 
                         w_kx2_ * x2_minus_y2 + w_ky2_ * two_x_y + 
                         k_off_;     
                         
    // 踝关节：线性组合
    double target_ankle = w_ax_ * x_ + w_ay_ * y_ + a_off_;  
    
    double target_camera = 0.0; // 锁定相机关节

    // ========================================================
    // 3. 打包发布 ROS 2 消息
    // ========================================================
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position = {target_hip1, target_hip2, target_hip3, target_knee, target_ankle, target_camera};

    publisher_->publish(msg);
  }

  // --- 节点私有成员变量 ---
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_subscriber_;
  std::vector<std::string> joint_names_;
  
  // 状态与时间变量
  double x_, y_, dt_ctrl_;
  
  // CPG 动力学参数
  double mu_, omega_, gamma_;
  
  // 髋关节映射参数
  double w_hx_, w_hy_, h_off_;
  // 膝关节映射参数
  double w_kx1_, w_ky1_, w_kx2_, w_ky2_, k_off_;
  // 踝关节映射参数
  double w_ax_, w_ay_, a_off_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SingleLegCPGNode>());
  rclcpp::shutdown();
  return 0;
}