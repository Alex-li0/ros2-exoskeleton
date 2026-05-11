#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <map>
#include <vector>
#include <string>
#include <cmath>

using namespace std::chrono_literals;

// 关节谐波模型：存储每个关节的权重和配置
struct JointModel
{
    int num_harmonics;       // 2: 常数+基频, 3: 常数+基频+2倍频...
    double w_offset;         // w0
    std::vector<double> w_x; // 各阶 cos 权重
    std::vector<double> w_y; // 各阶 sin 权重
    double limit_min, limit_max;
};

class AdaptiveCPGNode : public rclcpp::Node
{
public:
    AdaptiveCPGNode() : Node("adaptive_cpg_node")
    {
        // 1. 声明全局学习与物理参数
        this->declare_parameter("mu", 1.0);
        this->declare_parameter("gamma", 10.0);
        this->declare_parameter("omega", 6.28);      // 初始步频 (1Hz)
        this->declare_parameter("eta_w", 0.01);      // 权重学习率
        this->declare_parameter("eta_omega", 0.005); // 频率学习率

        // 2. 配置关节列表
        joint_names_ = {"left_hip3", "left_knee", "left_ankle", "right_hip3", "right_knee", "right_ankle"};
        static_joint_names_ = {"left_hip1", "left_hip2", "right_hip1", "right_hip2","left_camera","right_camera"};
        for (const auto &name : joint_names_)
        {
            std::string prefix = "joints." + name + ".";
            this->declare_parameter(prefix + "num_harmonics", 3);
            this->declare_parameter(prefix + "limit_min", -1.5);
            this->declare_parameter(prefix + "limit_max", 1.5);

            // === 新增：声明权重数组参数 ===
            // 默认给一个全0的数组，长度为 num_harmonics
            int n_h = this->get_parameter(prefix + "num_harmonics").as_int();
            this->declare_parameter(prefix + "w_offset", 0.0);
            this->declare_parameter(prefix + "w_x", std::vector<double>(n_h, 0.0));
            this->declare_parameter(prefix + "w_y", std::vector<double>(n_h, 0.0));
            // ============================

            JointModel m;
            m.num_harmonics = n_h;
            m.limit_min = this->get_parameter(prefix + "limit_min").as_double();
            m.limit_max = this->get_parameter(prefix + "limit_max").as_double();

            // === 新增：从参数服务器读取初始权重 ===
            m.w_offset = this->get_parameter(prefix + "w_offset").as_double();
            m.w_x = this->get_parameter(prefix + "w_x").as_double_array();
            m.w_y = this->get_parameter(prefix + "w_y").as_double_array();
            // ====================================
            RCLCPP_INFO(this->get_logger(), "关节 %s 加载完成: w_x[1]=%.4f, w_y[1]=%.4f",
                        name.c_str(), m.w_x[1], m.w_y[1]);
            joints_[name] = m;
        }

        // 3. TF 监听与发布者初始化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cpg_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // 4. 初始化状态
        x_ = 0.1;
        y_ = 0.0; // 初始扰动
        update_params();

        timer_ = this->create_wall_timer(10ms, std::bind(&AdaptiveCPGNode::on_timer, this));
        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "自适应傅里叶 CPG 节点已启动。");
    }

private:
    void on_timer()
{
    // --- 1. 计算真实的动态 dt ---
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // 安全保护：防止断点调试或系统卡顿时 dt 过大导致积分爆炸
    if (dt <= 0.0 || dt > 0.1) {
        dt = 0.01; 
    }

    update_params();

    // --- 2. 主振荡器 (Hopf Oscillator) 数值积分 ---
    double r2 = x_ * x_ + y_ * y_;
    double dx = (gamma_ * (mu_ - r2) * x_ - omega_ * y_) * dt;
    double dy = (gamma_ * (mu_ - r2) * y_ + omega_ * x_) * dt;
    x_ += dx;
    y_ += dy;

    // --- 3. 线性归一化的高阶谐波生成 (修复非线性放大隐患) ---
    std::vector<double> hx(5, 0.0), hy(5, 0.0);
    double r = std::sqrt(x_ * x_ + y_ * y_);
    
    // 防止奇异点除零错
    double r_safe = (r > 1e-5) ? r : 1e-5; 
    
    // 提取纯相位信息（单位圆上的坐标，幅值为 1）
    double x_n = x_ / r_safe; 
    double y_n = y_ / r_safe; 

    // 基频保持原样（幅值为 r）
    hx[1] = x_; 
    hy[1] = y_; 

    if (max_h_ >= 2)
    { 
        // n=2: 原公式幅值为 r^2, 除以 r_safe 后幅值降维至 r
        hx[2] = (x_ * x_ - y_ * y_) / r_safe;
        hy[2] = (2.0 * x_ * y_) / r_safe;
    }
    if (max_h_ >= 3)
    { 
        // n=3: 直接使用纯相位坐标计算，最后统一乘以 r，保证幅值为 r
        hx[3] = r * (x_n * (x_n * x_n - 3.0 * y_n * y_n));
        hy[3] = r * (y_n * (3.0 * x_n * x_n - y_n * y_n));
    }

    // --- 4. 遍历关节：计算输出 -> 获取反馈 -> 执行学习 ---
    // (这部分代码逻辑与您原来完全一致，但底层的 hx, hy 已经变得数学安全)
    auto cmd_msg = std_msgs::msg::Float64MultiArray();
    auto js_msg = sensor_msgs::msg::JointState();
    js_msg.header.stamp = now; // 使用刚才获取的准确系统时间

    for (const auto &name : joint_names_)
    {
        auto &m = joints_[name];
        bool is_right = (name.find("right") != std::string::npos);

        double theta_cpg = m.w_offset;
        for (int i = 1; i < m.num_harmonics; ++i)
        {
            double sign = (is_right && i % 2 != 0) ? -1.0 : 1.0;
            theta_cpg += m.w_x[i] * (hx[i] * sign) + m.w_y[i] * (hy[i] * sign);
        }

        double theta_target = get_target_from_tf(name, theta_cpg);
        double error = theta_target - theta_cpg;

        m.w_offset += eta_w_ * error * dt;
        for (int i = 1; i < m.num_harmonics; ++i)
        {
            double sign = (is_right && i % 2 != 0) ? -1.0 : 1.0;
            m.w_x[i] += eta_w_ * error * (hx[i] * sign) * dt;
            m.w_y[i] += eta_w_ * error * (hy[i] * sign) * dt;
        }

        if (name == "left_hip3")
        {
            omega_ += eta_omega_ * error * hy[1] * dt;
        }

        double output = std::clamp(theta_cpg, m.limit_min, m.limit_max);
        cmd_msg.data.push_back(output);
        js_msg.name.push_back(name);
        js_msg.position.push_back(output);
    }

    for (const auto &static_name : static_joint_names_)
    {
        js_msg.name.push_back(static_name);
        js_msg.position.push_back(0.0);
    }

    cpg_pub_->publish(cmd_msg);
    joint_state_pub_->publish(js_msg);
}

    double get_target_from_tf(const std::string &joint_name, double current_cpg)
    {
        // 在实际理想仿真中，如果没有外部参考，返回 current_cpg 使得 error=0
        // 若要演示学习功能，此处可改为查找 TF 树中名为 "ref_" + joint_name 的坐标变换
        try
        {
            // 伪代码逻辑：读取外部输入 frame
            // auto transform = tf_buffer_->lookupTransform("base_link", "ref_" + joint_name, ...);
            // return extract_angle(transform);
            return current_cpg;
        }
        catch (...)
        {
            return current_cpg;
        }
    }

    void update_params()
    {
        mu_ = this->get_parameter("mu").as_double();
        gamma_ = this->get_parameter("gamma").as_double();
        omega_ = this->get_parameter("omega").as_double();
        eta_w_ = this->get_parameter("eta_w").as_double();
        eta_omega_ = this->get_parameter("eta_omega").as_double();

        max_h_ = 1;
        for (auto &pair : joints_)
            max_h_ = std::max(max_h_, pair.second.num_harmonics - 1);
    }

    // 状态与参数
    double x_, y_, omega_;
    double mu_, gamma_, eta_w_, eta_omega_;
    int max_h_;
    std::map<std::string, JointModel> joints_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> static_joint_names_; // 静态关节列表
    

    // ROS 2 基础设施
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cpg_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptiveCPGNode>());
    rclcpp::shutdown();
    return 0;
}