#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

// ==================== CPG 状态机枚举 ====================
// IDLE     : 静止态，输出全零（各关节在 w_offset 位置）
// STARTING : 启动态，振幅从 0 平滑上升到 1（渐入）
// WALKING  : 行走态，振幅 = 1，全振幅输出
// STOPPING : 停止态，振幅从 1 平滑下降到 0（渐出）
enum class CPGState
{
    IDLE,
    STARTING,
    WALKING,
    STOPPING
};

// ==================== 关节谐波模型 ====================
struct JointModel
{
    int num_harmonics;       // 2: 常数+基频, 3: 常数+基频+2倍频...
    double w_offset;         // w0 — 也是站立静止时的关节目标角
    std::vector<double> w_x; // 各阶 cos 权重
    std::vector<double> w_y; // 各阶 sin 权重
    double limit_min, limit_max;
};

// ==================== 自适应 CPG 节点 ====================
class AdaptiveCPGNode : public rclcpp::Node
{
public:
    AdaptiveCPGNode() : Node("adaptive_cpg_node")
    {
        // 1. 声明全局学习与物理参数
        this->declare_parameter("mu", 1.0);
        this->declare_parameter("gamma", 10.0);
        this->declare_parameter("omega", 6.28);          // 初始步频 (1Hz)
        this->declare_parameter("eta_w", 0.01);           // 权重学习率
        this->declare_parameter("eta_omega", 0.005);      // 频率学习率
        this->declare_parameter("ramp_time", 0.5);        // 渐入/渐出时间 (秒)
        this->declare_parameter("initial_perturbation", 0.1); // 启动时振子扰动量
        this->declare_parameter("publish_ankle", true);   // 踝关节 CPG 输出开关

        // 2. 配置关节列表
        joint_names_ = {"left_hip3", "left_knee", "left_ankle",
                        "right_hip3", "right_knee", "right_ankle"};
        static_joint_names_ = {"left_hip1", "left_hip2",
                               "right_hip1", "right_hip2",
                               "left_camera", "right_camera"};
        for (const auto &name : joint_names_)
        {
            std::string prefix = "joints." + name + ".";
            this->declare_parameter(prefix + "num_harmonics", 3);
            this->declare_parameter(prefix + "limit_min", -1.5);
            this->declare_parameter(prefix + "limit_max", 1.5);

            int n_h = this->get_parameter(prefix + "num_harmonics").as_int();
            this->declare_parameter(prefix + "w_offset", 0.0);
            this->declare_parameter(prefix + "w_x", std::vector<double>(n_h, 0.0));
            this->declare_parameter(prefix + "w_y", std::vector<double>(n_h, 0.0));

            JointModel m;
            m.num_harmonics = n_h;
            m.limit_min = this->get_parameter(prefix + "limit_min").as_double();
            m.limit_max = this->get_parameter(prefix + "limit_max").as_double();
            m.w_offset = this->get_parameter(prefix + "w_offset").as_double();
            m.w_x = this->get_parameter(prefix + "w_x").as_double_array();
            m.w_y = this->get_parameter(prefix + "w_y").as_double_array();

            RCLCPP_INFO(this->get_logger(), "关节 %s 加载完成: w_x[1]=%.4f, w_y[1]=%.4f",
                        name.c_str(), m.w_x[1], m.w_y[1]);
            joints_[name] = m;
        }

        // 3. TF 监听与发布者初始化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cpg_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        // 4. 启停状态话题
        state_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cpg/state_cmd", 10,
            std::bind(&AdaptiveCPGNode::state_cmd_callback, this, std::placeholders::_1));
        state_pub_ = this->create_publisher<std_msgs::msg::String>("/cpg/state", 10);

        // 5. 初始化状态机
        cpg_state_ = CPGState::IDLE;
        amplitude_factor_ = 0.0;
        stop_start_amplitude_ = 1.0;
        ramp_start_time_ = this->now();

        // 6. 初始化振荡器（IDLE 状态下冻结为零态）
        x_ = 0.0;
        y_ = 0.0;
        update_params();

        timer_ = this->create_wall_timer(10ms,
            std::bind(&AdaptiveCPGNode::on_timer, this));
        last_time_ = this->now();

        publish_state("IDLE");
        RCLCPP_INFO(this->get_logger(),
            "自适应傅里叶 CPG 节点已启动 (带启停状态机)。ramp_time=%.2fs", ramp_time_);
    }

private:
    // ==================== 主定时器回调 ====================
    void on_timer()
    {
        // --- 1. 计算 dt ---
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0 || dt > 0.1) dt = 0.01;

        update_params();

        // --- 2. IDLE 状态：冻结振荡器，输出 0 位 (站立静止) ---
        if (cpg_state_ == CPGState::IDLE)
        {
            auto cmd_msg = std_msgs::msg::Float64MultiArray();
            for (size_t i = 0; i < joint_names_.size(); ++i)
                cmd_msg.data.push_back(0.0);

            auto js_msg = sensor_msgs::msg::JointState();
            js_msg.header.stamp = now;
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                js_msg.name.push_back(joint_names_[i]);
                js_msg.position.push_back(0.0);
            }
            for (const auto &name : static_joint_names_)
            {
                js_msg.name.push_back(name);
                js_msg.position.push_back(0.0);
            }
            cpg_pub_->publish(cmd_msg);
            joint_state_pub_->publish(js_msg);
            return;
        }

        // --- 3. 振荡器自由运行（STARTING / WALKING / STOPPING 均运行） ---
        // 使用子步长积分 (与 optimizer 的 sample_cpg_one_cycle 完全一致)
        // 必须用显式欧拉: dx,dy 用同一组 (x,y) 计算后再同时更新
        {
            int sub_steps = 10;
            double dt_sim = dt / sub_steps;
            for (int s = 0; s < sub_steps; ++s)
            {
                double r2 = x_ * x_ + y_ * y_;
                double dx = gamma_ * (mu_ - r2) * x_ - omega_ * y_;
                double dy = gamma_ * (mu_ - r2) * y_ + omega_ * x_;
                x_ += dx * dt_sim;
                y_ += dy * dt_sim;
            }
        }

        // --- 4. 计算振幅因子 A(t) ---
        amplitude_factor_ = compute_amplitude_factor();

        // --- 5. 状态转换检测 ---
        if (cpg_state_ == CPGState::STARTING && amplitude_factor_ >= 0.999)
        {
            amplitude_factor_ = 1.0;
            cpg_state_ = CPGState::WALKING;
            publish_state("WALKING");
            RCLCPP_INFO(this->get_logger(), "CPG 已完全启动，进入 WALKING 状态");
        }
        else if (cpg_state_ == CPGState::STOPPING && amplitude_factor_ <= 0.001)
        {
            amplitude_factor_ = 0.0;
            cpg_state_ = CPGState::IDLE;
            // 冻结振荡器到零态，防止下次 STARTING 时出现不连续
            x_ = 0.0;
            y_ = 0.0;
            publish_state("IDLE");
            RCLCPP_INFO(this->get_logger(), "CPG 已完全停止，进入 IDLE 状态");
        }

        // --- 6. 高阶谐波生成 ---
        // 与 optimizer 的 OLS 特征矩阵完全一一对应:
        //   n=1: X, Y                          (基频)
        //   n=2: X²-Y², 2XY                    (2倍频)
        //   n=3: X(X²-3Y²), Y(3X²-Y²)         (3倍频)
        std::vector<double> hx(5, 0.0), hy(5, 0.0);

        // 基频 n=1: 直接使用振荡器状态
        hx[1] = x_;
        hy[1] = y_;

        // 2倍频 n=2: X²-Y² = r²cos(2θ), 2XY = r²sin(2θ)
        if (max_h_ >= 2)
        {
            hx[2] = x_ * x_ - y_ * y_;
            hy[2] = 2.0 * x_ * y_;
        }

        // 3倍频 n=3: X(X²-3Y²) = r³cos(3θ), Y(3X²-Y²) = r³sin(3θ)
        if (max_h_ >= 3)
        {
            hx[3] = x_ * (x_ * x_ - 3.0 * y_ * y_);
            hy[3] = y_ * (3.0 * x_ * x_ - y_ * y_);
        }

        // --- 7. 遍历关节：计算输出 -> 获取反馈 -> 学习 ---
        auto cmd_msg = std_msgs::msg::Float64MultiArray();
        auto js_msg = sensor_msgs::msg::JointState();
        js_msg.header.stamp = now;

        for (const auto &name : joint_names_)
        {
            auto &m = joints_[name];
            bool is_right = (name.find("right") != std::string::npos);

            // 计算全振幅 CPG 输出
            double theta_cpg = m.w_offset;
            for (int i = 1; i < m.num_harmonics; ++i)
            {
                double sign = (is_right && i % 2 != 0) ? -1.0 : 1.0;
                theta_cpg += m.w_x[i] * (hx[i] * sign) +
                             m.w_y[i] * (hy[i] * sign);
            }

            // 学习（eta_w=0 时不影响输出）
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
                omega_ += eta_omega_ * error * hy[1] * dt;

            // ====== 核心：振幅调制 ======
            // output = A(t) * theta_cpg
            // A=0 -> output=0 (站立 0 位)
            // A=1 -> output=theta_cpg (全振幅步态, 包含 w_offset 振荡中心)
            double output = amplitude_factor_ * theta_cpg;
            output = std::clamp(output, m.limit_min, m.limit_max);

            // 踝关节开关: 实机时 cmd 锁定 0 位，仿真时正常输出 CPG
            bool is_ankle = (name.find("ankle") != std::string::npos);
            double cmd_output = output;
            if (is_ankle && !publish_ankle_)
            {
                // 实机模式: 踝关节不助力，cmd 输出锁定在 0 位
                cmd_output = 0.0;
            }

            cmd_msg.data.push_back(cmd_output);
            js_msg.name.push_back(name);
            js_msg.position.push_back(output);  // joint_states 始终显示 CPG 曲线
        }

        for (const auto &static_name : static_joint_names_)
        {
            js_msg.name.push_back(static_name);
            js_msg.position.push_back(0.0);
        }

        cpg_pub_->publish(cmd_msg);
        joint_state_pub_->publish(js_msg);
    }

    // ==================== 启停指令回调 ====================
    void state_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = msg->data;
        // 去除首尾空白
        cmd.erase(0, cmd.find_first_not_of(" \t\n\r"));
        cmd.erase(cmd.find_last_not_of(" \t\n\r") + 1);

        if (cmd == "start" || cmd == "START")
        {
            if (cpg_state_ == CPGState::IDLE)
            {
                // 从 IDLE → STARTING：扰动振子，使其脱离零态
                double pert = this->get_parameter("initial_perturbation").as_double();
                x_ = std::sqrt(mu_) * pert; // 确保非零起始
                y_ = 0.0;
                cpg_state_ = CPGState::STARTING;
                amplitude_factor_ = 0.0;
                ramp_start_time_ = this->now();
                publish_state("STARTING");
                RCLCPP_INFO(this->get_logger(), "CPG 启动：开始渐入 (ramp_time=%.2fs)", ramp_time_);
            }
            else if (cpg_state_ == CPGState::STOPPING)
            {
                // 停止中反悔 → 重新启动：从当前 A 反向渐入
                cpg_state_ = CPGState::STARTING;
                ramp_start_time_ = this->now();
                publish_state("STARTING");
                RCLCPP_INFO(this->get_logger(), "CPG 重新启动：从 A=%.3f 开始渐入", amplitude_factor_);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "CPG 当前状态非 IDLE/STOPPING，忽略 start 指令");
            }
        }
        else if (cmd == "stop" || cmd == "STOP")
        {
            if (cpg_state_ == CPGState::WALKING || cpg_state_ == CPGState::STARTING)
            {
                cpg_state_ = CPGState::STOPPING;
                stop_start_amplitude_ = amplitude_factor_;
                ramp_start_time_ = this->now();
                publish_state("STOPPING");
                RCLCPP_INFO(this->get_logger(), "CPG 停止：开始渐出 (A=%.3f)", amplitude_factor_);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "CPG 当前状态非 WALKING/STARTING，忽略 stop 指令");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "未知的 CPG 指令: '%s' (支持: start/stop)", cmd.c_str());
        }
    }

    // ==================== 振幅因子计算 ====================
    // 使用半余弦窗函数实现 C1 连续（无速度跳变）
    //   A(t) = 0.5 * (1 - cos(π * t/T))   渐入: 0→1
    //   A(t) = 0.5 * (1 + cos(π * t/T))   渐出: 1→0
    double compute_amplitude_factor()
    {
        double elapsed = (this->now() - ramp_start_time_).seconds();
        double T = ramp_time_;
        if (T < 0.01) T = 0.01; // 最小保护

        switch (cpg_state_)
        {
        case CPGState::IDLE:
            return 0.0;

        case CPGState::STARTING:
        {
            double t = std::clamp(elapsed / T, 0.0, 1.0);
            // 半余弦渐入: A(0)=0, A(T)=1, dA/dt 在两端均为 0
            return 0.5 * (1.0 - std::cos(M_PI * t));
        }

        case CPGState::WALKING:
            return 1.0;

        case CPGState::STOPPING:
        {
            double t = std::clamp(elapsed / T, 0.0, 1.0);
            // 从 stop_start_amplitude_ 平滑衰减到 0
            // A(t) = stop_start_amplitude_ * 0.5 * (1 + cos(π * t))
            return stop_start_amplitude_ * 0.5 * (1.0 + std::cos(M_PI * t));
        }

        default:
            return 0.0;
        }
    }

    // ==================== 发布状态 ====================
    void publish_state(const std::string &state_str)
    {
        auto msg = std_msgs::msg::String();
        msg.data = state_str;
        state_pub_->publish(msg);
    }

    // ==================== 从 TF 获取目标角度 ====================
    double get_target_from_tf(const std::string &joint_name, double current_cpg)
    {
        try
        {
            return current_cpg;
        }
        catch (...)
        {
            return current_cpg;
        }
    }

    // ==================== 参数更新 ====================
    void update_params()
    {
        mu_ = this->get_parameter("mu").as_double();
        gamma_ = this->get_parameter("gamma").as_double();
        omega_ = this->get_parameter("omega").as_double();
        eta_w_ = this->get_parameter("eta_w").as_double();
        eta_omega_ = this->get_parameter("eta_omega").as_double();
        ramp_time_ = this->get_parameter("ramp_time").as_double();
        publish_ankle_ = this->get_parameter("publish_ankle").as_bool();

        max_h_ = 1;
        for (auto &pair : joints_)
            max_h_ = std::max(max_h_, pair.second.num_harmonics - 1);
    }

    // ==================== 成员变量 ====================
    // 振荡器状态
    double x_, y_, omega_;
    double mu_, gamma_, eta_w_, eta_omega_;
    int max_h_;

    // 启停状态机
    CPGState cpg_state_;
    double amplitude_factor_;       // A(t) ∈ [0, 1]
    double stop_start_amplitude_;   // 进入 STOPPING 瞬间的 A 值
    double ramp_time_;              // 渐入/渐出时间常数
    rclcpp::Time ramp_start_time_;  // 当前渐变阶段的起始时刻

    // 踝关节开关
    bool publish_ankle_;

    // 关节模型
    std::map<std::string, JointModel> joints_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> static_joint_names_;

    // ROS 2 基础设施
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cpg_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_cmd_sub_;
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