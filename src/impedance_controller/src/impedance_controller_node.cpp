#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

class ImpedanceControllerNode : public rclcpp::Node
{
public:
    ImpedanceControllerNode() : Node("impedance_controller_node")
    {
        // 1. 声明参数
        this->declare_parameter("joint_names", std::vector<std::string>{});
        this->declare_parameter("K", std::vector<double>{});
        this->declare_parameter("D", std::vector<double>{});
        this->declare_parameter("torque_limit", std::vector<double>{});
        this->declare_parameter("torque_rate_limit", std::vector<double>{});
        this->declare_parameter("feedback_topic", std::string("/joint_states/hardware"));
        this->declare_parameter("use_mock_feedback", true);
        this->declare_parameter("control_frequency", 500.0);
        this->declare_parameter("velocity_filter_alpha", 0.2);
        this->declare_parameter("data_timeout", 0.01);

        // 2. 加载参数
        joint_names_ = this->get_parameter("joint_names").as_string_array();
        K_ = this->get_parameter("K").as_double_array();
        D_ = this->get_parameter("D").as_double_array();
        torque_limit_ = this->get_parameter("torque_limit").as_double_array();
        torque_rate_limit_ = this->get_parameter("torque_rate_limit").as_double_array();
        use_mock_feedback_ = this->get_parameter("use_mock_feedback").as_bool();
        double freq = this->get_parameter("control_frequency").as_double();
        vel_filter_alpha_ = this->get_parameter("velocity_filter_alpha").as_double();
        data_timeout_ = this->get_parameter("data_timeout").as_double();

        num_joints_ = joint_names_.size();

        // 参数校验
        if (K_.size() != num_joints_ || D_.size() != num_joints_ ||
            torque_limit_.size() != num_joints_ || torque_rate_limit_.size() != num_joints_)
        {
            RCLCPP_ERROR(this->get_logger(), "参数维度不匹配: joints=%zu, K=%zu, D=%zu, limit=%zu, rate=%zu",
                         num_joints_, K_.size(), D_.size(), torque_limit_.size(), torque_rate_limit_.size());
            throw std::runtime_error("Parameter dimension mismatch");
        }

        // 3. 初始化内部状态
        q_ref_.assign(num_joints_, 0.0);
        qdot_ref_.assign(num_joints_, 0.0);
        q_meas_.assign(num_joints_, 0.0);
        qdot_meas_.assign(num_joints_, 0.0);
        qdot_filt_.assign(num_joints_, 0.0);
        tau_prev_.assign(num_joints_, 0.0);
        tau_cmd_.assign(num_joints_, 0.0);

        cpg_data_valid_ = false;
        feedback_data_valid_ = false;
        last_cpg_time_ = this->now();
        last_feedback_time_ = this->now();

        // 4. 创建订阅
        cpg_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/cpg/joint_ref", 10,
            std::bind(&ImpedanceControllerNode::cpg_callback, this, std::placeholders::_1));

        if (!use_mock_feedback_)
        {
            auto fb_topic = this->get_parameter("feedback_topic").as_string();
            feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                fb_topic, 10,
                std::bind(&ImpedanceControllerNode::feedback_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "订阅硬件反馈话题: %s", fb_topic.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "仿真模式: 使用 CPG 参考作为反馈（输出零力矩）");
        }

        // 5. 创建发布器
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/torque_commands", 10);
        debug_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/impedance/debug", 10);

        // 6. 创建控制定时器
        auto period = std::chrono::duration<double>(1.0 / freq);
        control_timer_ = this->create_wall_timer(
            period, std::bind(&ImpedanceControllerNode::control_callback, this));

        RCLCPP_INFO(this->get_logger(),
            "阻抗控制器已启动: %zu 关节, %.0f Hz, mock=%s",
            num_joints_, freq, use_mock_feedback_ ? "true" : "false");

        for (size_t i = 0; i < num_joints_; ++i)
        {
            RCLCPP_INFO(this->get_logger(), "  [%zu] %s: K=%.1f, D=%.1f, limit=%.1f N*m",
                        i, joint_names_[i].c_str(), K_[i], D_[i], torque_limit_[i]);
        }
    }

private:
    // ==================== CPG 参考回调 ====================
    void cpg_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < num_joints_; ++i)
        {
            for (size_t j = 0; j < msg->name.size(); ++j)
            {
                if (msg->name[j] == joint_names_[i])
                {
                    q_ref_[i] = msg->position[j];
                    if (j < msg->velocity.size())
                        qdot_ref_[i] = msg->velocity[j];
                    break;
                }
            }
        }
        last_cpg_time_ = this->now();
        cpg_data_valid_ = true;
    }

    // ==================== 硬件反馈回调 ====================
    void feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < num_joints_; ++i)
        {
            for (size_t j = 0; j < msg->name.size(); ++j)
            {
                if (msg->name[j] == joint_names_[i])
                {
                    q_meas_[i] = msg->position[j];
                    if (j < msg->velocity.size())
                        qdot_meas_[i] = msg->velocity[j];
                    break;
                }
            }
        }
        last_feedback_time_ = this->now();
        feedback_data_valid_ = true;
    }

    // ==================== 控制循环 ====================
    void control_callback()
    {
        auto now = this->now();
        double dt = 1.0 / this->get_parameter("control_frequency").as_double();

        // 1. 检查 CPG 数据新鲜度
        if ((now - last_cpg_time_).seconds() > data_timeout_)
        {
            if (cpg_data_valid_)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "CPG 数据超时 (>%.0fms)，输出零力矩", data_timeout_ * 1000);
                cpg_data_valid_ = false;
            }
            publish_zero_torque();
            return;
        }

        // 2. 仿真模式：用 CPG 参考当反馈
        if (use_mock_feedback_)
        {
            q_meas_ = q_ref_;
            qdot_meas_ = qdot_ref_;
        }
        else
        {
            // 检查硬件反馈新鲜度
            if ((now - last_feedback_time_).seconds() > data_timeout_)
            {
                if (feedback_data_valid_)
                {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "硬件反馈超时 (>%.0fms)，输出零力矩", data_timeout_ * 1000);
                    feedback_data_valid_ = false;
                }
                publish_zero_torque();
                return;
            }

            // 速度低通滤波
            for (size_t i = 0; i < num_joints_; ++i)
            {
                qdot_filt_[i] = vel_filter_alpha_ * qdot_meas_[i] +
                                (1.0 - vel_filter_alpha_) * qdot_filt_[i];
            }
        }

        // 3. 计算阻抗力矩
        auto torque_msg = std_msgs::msg::Float64MultiArray();
        auto debug_msg = sensor_msgs::msg::JointState();
        debug_msg.header.stamp = now;

        for (size_t i = 0; i < num_joints_; ++i)
        {
            double q_dot_meas = use_mock_feedback_ ? qdot_meas_[i] : qdot_filt_[i];

            double pos_error = q_ref_[i] - q_meas_[i];
            double vel_error = qdot_ref_[i] - q_dot_meas;

            // τ = K * (q_ref - q) + D * (q̇_ref - q̇)
            double tau_raw = K_[i] * pos_error + D_[i] * vel_error;

            // 变化率限制
            double max_delta = torque_rate_limit_[i] * dt;
            double delta = tau_raw - tau_prev_[i];
            delta = std::clamp(delta, -max_delta, max_delta);
            double tau = tau_prev_[i] + delta;

            // 力矩限幅
            tau = std::clamp(tau, -torque_limit_[i], torque_limit_[i]);

            tau_cmd_[i] = tau;
            tau_prev_[i] = tau;
            torque_msg.data.push_back(tau);

            // 调试信息
            debug_msg.name.push_back(joint_names_[i]);
            debug_msg.position.push_back(pos_error);
            debug_msg.velocity.push_back(vel_error);
            debug_msg.effort.push_back(tau);
        }

        torque_pub_->publish(torque_msg);
        debug_pub_->publish(debug_msg);
    }

    // ==================== 发布零力矩 ====================
    void publish_zero_torque()
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.assign(num_joints_, 0.0);
        torque_pub_->publish(msg);
        tau_prev_.assign(num_joints_, 0.0);
    }

    // ==================== 成员变量 ====================
    size_t num_joints_;
    std::vector<std::string> joint_names_;
    std::vector<double> K_, D_;
    std::vector<double> torque_limit_, torque_rate_limit_;
    double vel_filter_alpha_;
    double data_timeout_;
    bool use_mock_feedback_;

    std::vector<double> q_ref_, qdot_ref_;
    std::vector<double> q_meas_, qdot_meas_, qdot_filt_;
    std::vector<double> tau_prev_, tau_cmd_;

    bool cpg_data_valid_;
    bool feedback_data_valid_;
    rclcpp::Time last_cpg_time_;
    rclcpp::Time last_feedback_time_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cpg_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr debug_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImpedanceControllerNode>());
    rclcpp::shutdown();
    return 0;
}
