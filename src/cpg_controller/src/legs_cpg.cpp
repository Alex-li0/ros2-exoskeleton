#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "cpg_controller/reference_math.hpp"
#include "cpg_controller/learning_interface.hpp"
#include <terrain_adaptive_cpg/msg/cpg_debug.hpp>
#include <terrain_adaptive_cpg/msg/cpg_parameter_target.hpp>
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;
using cpg_controller::Sample;

// ==================== CPG 状态机枚举 ====================
// IDLE      : 静止态，输出全零（不等同于控制器零电流）
// STARTING  : 启动态，振幅从 0 平滑上升到 1（渐入）
// WALKING   : 行走态，振幅 = 1，全振幅输出
// STOPPING  : 停止态，振幅从 1 平滑下降到 0（渐出）
// SWITCHING : 切换态，在两个 gait mode 之间平滑过渡（保持全振幅）
enum class CPGState
{
    IDLE,
    STARTING,
    WALKING,
    STOPPING,
    SWITCHING
};

// ==================== 关节谐波模型 ====================
struct JointModel
{
    int num_harmonics;       // 2: 常数+基频, 3: 常数+基频+2倍频...
    double w_offset;         // 周期轨迹的直流分量；IDLE 输出为零
    std::vector<double> w_x; // 各阶 cos 权重
    std::vector<double> w_y; // 各阶 sin 权重
    double limit_min, limit_max;
};

// ==================== 用于切换过渡的权重快照 ====================
struct JointWeightSnapshot
{
    double w_offset;
    std::vector<double> w_x;
    std::vector<double> w_y;
    double limit_min;
    double limit_max;
};

// ==================== Gait Mode ====================
struct GaitMode
{
    double omega;
    std::map<std::string, JointModel> joints;
};

// ==================== 自适应 CPG 节点 ====================
class AdaptiveCPGNode : public rclcpp::Node
{
public:
    AdaptiveCPGNode() : Node("adaptive_cpg_node")
    {
        // 1. 声明全局参数（不含 omega，omega 属于每个 gait mode）
        this->declare_parameter("mu", 1.0);
        this->declare_parameter("gamma", 10.0);
        this->declare_parameter("eta_w", 0.0);
        this->declare_parameter("eta_omega", 0.0);
        this->declare_parameter("ramp_time", 0.5);
        this->declare_parameter("initial_perturbation", 0.1);
        this->declare_parameter("publish_ankle", true);
        this->declare_parameter("parameter_target_timeout_s", 0.0);
        this->declare_parameter("modulation_transition_s", 0.5);
        this->declare_parameter("observe_measurements", false);
        this->declare_parameter("measurement_timeout_s", 0.1);

        // 2. 加载 Gait Library
        joint_names_ = {"left_hip3", "left_knee", "left_ankle",
                        "right_hip3", "right_knee", "right_ankle"};
        static_joint_names_ = {"left_hip1", "left_hip2",
                               "right_hip1", "right_hip2",
                               "left_camera", "right_camera"};

        this->declare_parameter("default_gait", std::string("flat_walk"));
        this->declare_parameter("gait_modes", std::vector<std::string>{});

        auto mode_names = this->get_parameter("gait_modes").as_string_array();
        if (mode_names.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "gait_modes 列表为空！");
            throw std::runtime_error("No gait modes defined");
        }

        for (const auto &mode_name : mode_names)
        {
            std::string prefix = "gait_library." + mode_name + ".";
            this->declare_parameter(prefix + "omega", 6.28);

            GaitMode mode;
            mode.omega = this->get_parameter(prefix + "omega").as_double();
            if (!std::isfinite(mode.omega) || mode.omega <= 0.0)
                throw std::runtime_error("Invalid gait frequency: " + mode_name);

            for (const auto &jname : joint_names_)
            {
                std::string jp = prefix + "joints." + jname + ".";
                this->declare_parameter(jp + "num_harmonics", 3);
                this->declare_parameter(jp + "limit_min", -1.5);
                this->declare_parameter(jp + "limit_max", 1.5);

                int n_h = this->get_parameter(jp + "num_harmonics").as_int();
                if (n_h < 2 || n_h > 4)
                    throw std::runtime_error("num_harmonics must be 2..4: " + jp);
                this->declare_parameter(jp + "w_offset", 0.0);
                this->declare_parameter(jp + "w_x", std::vector<double>(n_h, 0.0));
                this->declare_parameter(jp + "w_y", std::vector<double>(n_h, 0.0));

                JointModel m;
                m.num_harmonics = n_h;
                m.limit_min = this->get_parameter(jp + "limit_min").as_double();
                m.limit_max = this->get_parameter(jp + "limit_max").as_double();
                m.w_offset = this->get_parameter(jp + "w_offset").as_double();
                m.w_x = this->get_parameter(jp + "w_x").as_double_array();
                m.w_y = this->get_parameter(jp + "w_y").as_double_array();
                if (m.w_x.size() != static_cast<size_t>(n_h) || m.w_y.size() != static_cast<size_t>(n_h) ||
                    !std::isfinite(m.w_offset) || !std::isfinite(m.limit_min) ||
                    !std::isfinite(m.limit_max) || m.limit_min >= m.limit_max ||
                    !std::all_of(m.w_x.begin(), m.w_x.end(), [](double v) { return std::isfinite(v); }) ||
                    !std::all_of(m.w_y.begin(), m.w_y.end(), [](double v) { return std::isfinite(v); }))
                    throw std::runtime_error("Invalid joint model: " + jp);
                mode.joints[jname] = m;
            }

            gait_library_[mode_name] = mode;
            RCLCPP_INFO(this->get_logger(), "Gait mode '%s' 加载完成: omega=%.3f, %zu 关节",
                        mode_name.c_str(), mode.omega, mode.joints.size());
        }

        auto default_gait = this->get_parameter("default_gait").as_string();
        if (gait_library_.find(default_gait) == gait_library_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "default_gait '%s' 不在 gait_modes 列表中",
                         default_gait.c_str());
            throw std::runtime_error("Invalid default_gait");
        }
        current_gait_name_ = default_gait;
        active_gait_ = gait_library_.at(current_gait_name_);
        current_gait_ = &active_gait_;
        RCLCPP_INFO(this->get_logger(), "默认 gait mode: '%s'", current_gait_name_.c_str());

        // 3. ROS 话题
        measurement_timeout_s_ = get_parameter("measurement_timeout_s").as_double();
        if (!std::isfinite(measurement_timeout_s_) || measurement_timeout_s_ <= 0.0)
            throw std::runtime_error("measurement_timeout_s must be positive");
        if (get_parameter("observe_measurements").as_bool())
        {
            // Deliberately distinct from /joint_states: display remapping must not affect input.
            measurement_sub_ = create_subscription<sensor_msgs::msg::JointState>(
                "/cpg/measured_joint_states", rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                    if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u)
                        return;
                    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
                    measurements_.accept(msg->name, msg->position, stamp, now().seconds(),
                        measurement_timeout_s_, joint_names_);
                });
        }

        cpg_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        cpg_ref_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/cpg/joint_ref", 10);

        if (measurement_sub_ &&
            (std::string(measurement_sub_->get_topic_name()) == joint_state_pub_->get_topic_name() ||
             std::string(measurement_sub_->get_topic_name()) == cpg_ref_pub_->get_topic_name()))
            throw std::runtime_error("Measurement input must not alias CPG reference/display output");

        state_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cpg/state_cmd", 10,
            std::bind(&AdaptiveCPGNode::state_cmd_callback, this, std::placeholders::_1));
        state_pub_ = this->create_publisher<std_msgs::msg::String>("/cpg/state", 10);
        switch_debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/cpg/switch_debug", 10);
        cpg_debug_pub_ = this->create_publisher<terrain_adaptive_cpg::msg::CPGDebug>("/cpg/debug", 10);

        gait_mode_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cpg/gait_mode_target", 10,
            std::bind(&AdaptiveCPGNode::gait_mode_cmd_callback, this, std::placeholders::_1));
        gait_mode_pub_ = this->create_publisher<std_msgs::msg::String>("/cpg/gait_mode", 10);
        parameter_target_sub_ = this->create_subscription<terrain_adaptive_cpg::msg::CPGParameterTarget>(
            "/cpg/parameter_target", 10,
            std::bind(&AdaptiveCPGNode::parameter_target_callback, this, std::placeholders::_1));

        // 4. 初始化状态机
        cpg_state_ = CPGState::IDLE;
        amplitude_factor_ = 0.0;
        ramp_start_time_ = this->now();

        // 5. 初始化振荡器（IDLE 状态下冻结为零态）
        x_ = 0.0;
        y_ = 0.0;
        update_params();

        // 6. 创建定时器
        // CPG 积分定时器: 500Hz (2ms)
        cpg_timer_ = this->create_wall_timer(2ms,
            std::bind(&AdaptiveCPGNode::cpg_timer_callback, this));
        last_time_ = this->now();
        last_parameter_target_time_ = this->now();
        amplitude_scales_.assign(joint_names_.size(), 1.0);
        offset_deltas_.assign(joint_names_.size(), 0.0);

        publish_state("IDLE");
        RCLCPP_INFO(this->get_logger(),
            "自适应傅里叶 CPG 节点已启动 (带启停状态机)。ramp_time=%.2fs", ramp_time_);
    }

private:
    // ==================== CPG 积分定时器回调 (500Hz) ====================
    void cpg_timer_callback()
    {
        // --- 1. 计算 dt ---
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0 || dt > 0.1) dt = 0.002;  // 默认 2ms (500Hz)
        update_modulation_status(now);
        omega_scale_ = omega_modulation_.sample(now.seconds()).value;
        for (size_t j = 0; j < joint_names_.size(); ++j) {
            amplitude_scales_[j] = amplitude_modulation_[j].sample(now.seconds()).value;
            offset_deltas_[j] = offset_modulation_[j].sample(now.seconds()).value;
        }
        // Complete a mode transition independently of a concurrent stop envelope.
        if (switch_active_ && compute_switch_progress(now).value >= 1.0) {
            current_gait_name_ = switch_target_gait_;
            active_gait_ = *switch_target_gait_ptr_;
            omega_ = active_gait_.omega;
            switch_active_ = false;
            auto mode = std_msgs::msg::String(); mode.data = current_gait_name_;
            gait_mode_pub_->publish(mode);
            if (cpg_state_ == CPGState::SWITCHING) {
                cpg_state_ = CPGState::WALKING;
                publish_state("WALKING");
            }
        }

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

            // IDLE 时也发布 /cpg/joint_ref（position=0, velocity=0）
            auto ref_msg = sensor_msgs::msg::JointState();
            ref_msg.header.stamp = now;
            for (const auto &name : joint_names_)
            {
                ref_msg.name.push_back(name);
                ref_msg.position.push_back(0.0);
                ref_msg.velocity.push_back(0.0);
            }
            cpg_ref_pub_->publish(ref_msg);
            publish_structured_debug(now, 1.0, omega_, {}, {}, {}, {}, {}, {}, {}, {}, {});
            return;
        }

        // --- 3. 计算切换进度 & 插值 omega（必须在积分之前） ---
        Sample switch_sample{1.0, 0.0};
        double switch_alpha = 1.0;
        double omega_current = omega_;
        if (switch_active_)
        {
            switch_sample = compute_switch_progress(now);
            switch_alpha = switch_sample.value;
            omega_current = (1.0 - switch_alpha) * switch_old_omega_ + switch_alpha * switch_new_omega_;
        }
        omega_current *= omega_scale_;

        // --- 4. 振荡器自由运行 ---
        // 使用子步长积分 (与 optimizer 的 sample_cpg_one_cycle 完全一致)
        {
            int sub_steps = 10;
            double dt_sim = dt / sub_steps;
            for (int s = 0; s < sub_steps; ++s)
            {
                double r2 = x_ * x_ + y_ * y_;
                double dx = gamma_ * (mu_ - r2) * x_ - omega_current * y_;
                double dy = gamma_ * (mu_ - r2) * y_ + omega_current * x_;
                x_ += dx * dt_sim;
                y_ += dy * dt_sim;
            }
        }

        const double r2 = x_ * x_ + y_ * y_;
        const auto basis = cpg_controller::harmonics(x_, y_,
            gamma_ * (mu_ - r2) * x_ - omega_current * y_,
            gamma_ * (mu_ - r2) * y_ + omega_current * x_);
        const auto envelope = cpg_controller::constrain(amplitude_envelope_.sample(now.seconds()),
            {0.0, 0.0}, {1.0, 0.0});
        amplitude_factor_ = envelope.value;
        const double elapsed = (now - ramp_start_time_).seconds();
        if (cpg_state_ == CPGState::STARTING && elapsed >= ramp_time_) {
            cpg_state_ = CPGState::WALKING;
            publish_state("WALKING");
        } else if (cpg_state_ == CPGState::STOPPING && elapsed >= ramp_time_) {
            cpg_state_ = CPGState::IDLE;
            x_ = y_ = 0.0;
            publish_state("IDLE");
        }

        // --- 9. 遍历关节：计算输出 ---
        auto cmd_msg = std_msgs::msg::Float64MultiArray();
        auto js_msg = sensor_msgs::msg::JointState();
        js_msg.header.stamp = now;
        auto debug_msg = std_msgs::msg::Float64MultiArray();
        debug_msg.data.push_back(switch_alpha);   // [0]
        debug_msg.data.push_back(omega_current);   // [1]

        // 先收集每个关节的插值权重和输出
        std::vector<double> dbg_w_off, dbg_wx1, dbg_wx2, dbg_wx3;
        std::vector<double> dbg_wy1, dbg_wy2, dbg_wy3;
        std::vector<double> dbg_theta, dbg_output;
        std::vector<double> joint_velocities;  // 每个关节的速度

        for (const auto &name : joint_names_)
        {
            auto &m = current_gait_->joints[name];

            auto coefficients = [](const auto &model) {
                cpg_controller::Coefficients result;
                result.offset.value = model.w_offset;
                for (size_t n = 1; n <= 3; ++n) {
                    result.x[n].value = n < model.w_x.size() ? model.w_x[n] : 0.0;
                    result.y[n].value = n < model.w_y.size() ? model.w_y[n] : 0.0;
                }
                return result;
            };
            auto w = coefficients(m);
            Sample low{m.limit_min, 0.0}, high{m.limit_max, 0.0};
            if (switch_active_) {
                const auto &old = switch_old_weights_.at(name);
                const auto &next = switch_new_weights_.at(name);
                const auto a = coefficients(old), b = coefficients(next);
                w.offset = cpg_controller::blend(a.offset, b.offset, switch_sample);
                for (size_t n = 1; n <= 3; ++n) {
                    w.x[n] = cpg_controller::blend(a.x[n], b.x[n], switch_sample);
                    w.y[n] = cpg_controller::blend(a.y[n], b.y[n], switch_sample);
                }
                // Moving limits avoid a union-of-limits jump at transition completion.
                low = cpg_controller::blend({old.limit_min, 0}, {next.limit_min, 0}, switch_sample);
                high = cpg_controller::blend({old.limit_max, 0}, {next.limit_max, 0}, switch_sample);
            }
            const size_t joint_index = dbg_w_off.size();
            w.offset = w.offset + offset_modulation_[joint_index].sample(now.seconds());
            const auto scale = amplitude_modulation_[joint_index].sample(now.seconds());
            for (size_t n = 1; n <= 3; ++n) {
                w.x[n] = w.x[n] * scale;
                w.y[n] = w.y[n] * scale;
            }
            const auto theta = cpg_controller::readout(w, basis);
            auto reference = cpg_controller::constrain(envelope * theta, envelope * low, envelope * high);
            const bool is_ankle = name.find("ankle") != std::string::npos;
            if (is_ankle && !publish_ankle_) reference = {};
            const double output = reference.value, cmd_output = output;
            joint_velocities.push_back(reference.rate);
            const double theta_interp = theta.value;
            const double w_off_cur = w.offset.value;
            const double wx1 = w.x[1].value, wx2 = w.x[2].value, wx3 = w.x[3].value;
            const double wy1 = w.y[1].value, wy2 = w.y[2].value, wy3 = w.y[3].value;

            cmd_msg.data.push_back(cmd_output);
            js_msg.name.push_back(name);
            js_msg.position.push_back(output);

            dbg_w_off.push_back(w_off_cur);
            dbg_wx1.push_back(wx1); dbg_wx2.push_back(wx2); dbg_wx3.push_back(wx3);
            dbg_wy1.push_back(wy1); dbg_wy2.push_back(wy2); dbg_wy3.push_back(wy3);
            dbg_theta.push_back(theta_interp);
            dbg_output.push_back(output);
        }

        // debug layout: [2+j*7+0]=w_off, [+1]=wx1, [+2]=wx2, [+3]=wx3,
        //               [+4]=wy1, [+5]=wy2, [+6]=wy3, [2+42+j]=theta, [2+48+j]=output
        for (size_t j = 0; j < joint_names_.size(); ++j)
        {
            debug_msg.data.push_back(dbg_w_off[j]);
            debug_msg.data.push_back(dbg_wx1[j]);
            debug_msg.data.push_back(dbg_wx2[j]);
            debug_msg.data.push_back(dbg_wx3[j]);
            debug_msg.data.push_back(dbg_wy1[j]);
            debug_msg.data.push_back(dbg_wy2[j]);
            debug_msg.data.push_back(dbg_wy3[j]);
        }
        for (auto v : dbg_theta)  debug_msg.data.push_back(v);
        for (auto v : dbg_output) debug_msg.data.push_back(v);

        for (const auto &static_name : static_joint_names_)
        {
            js_msg.name.push_back(static_name);
            js_msg.position.push_back(0.0);
        }

        cpg_pub_->publish(cmd_msg);
        joint_state_pub_->publish(js_msg);
        switch_debug_pub_->publish(debug_msg);

        // 发布 /cpg/joint_ref（含位置和速度，供阻抗控制器使用）
        auto ref_msg = sensor_msgs::msg::JointState();
        ref_msg.header.stamp = now;
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            ref_msg.name.push_back(joint_names_[i]);
            ref_msg.position.push_back(cmd_msg.data[i]);
            ref_msg.velocity.push_back(joint_velocities[i]);
        }
        cpg_ref_pub_->publish(ref_msg);
        publish_structured_debug(now, switch_alpha, omega_current, dbg_w_off, dbg_wx1, dbg_wx2,
            dbg_wx3, dbg_wy1, dbg_wy2, dbg_wy3, dbg_output, joint_velocities);
    }

    void parameter_target_callback(const terrain_adaptive_cpg::msg::CPGParameterTarget::SharedPtr msg)
    {
        const auto received = this->now();
        if (msg->stamp.sec < 0 || msg->stamp.nanosec >= 1000000000u) return;
        const double stamp = rclcpp::Time(msg->stamp).seconds();
        if (!std::isfinite(stamp) || stamp <= 0.0 || stamp <= last_parameter_stamp_ ||
            stamp > received.seconds() || received.seconds() - stamp > parameter_target_timeout_s_)
            return;
        if (!msg->continuous_modulation_enabled) {
            last_parameter_stamp_ = stamp;
            reset_modulation(received.seconds());
            return;
        }
        if (parameter_target_timeout_s_ <= 0.0)
            return;
        if (!msg->omega_scale_valid || !std::isfinite(msg->omega_scale) || msg->omega_scale <= 0.0)
            return;
        if (msg->joint_names.size() != msg->amplitude_scale.size() ||
            msg->joint_names.size() != msg->amplitude_scale_valid.size() ||
            msg->joint_names.size() != msg->offset_delta.size() ||
            msg->joint_names.size() != msg->offset_delta_valid.size())
            return;

        if (msg->joint_names.size() != joint_names_.size()) return;
        for (const auto &name : msg->joint_names)
            if (std::count(msg->joint_names.begin(), msg->joint_names.end(), name) != 1) return;
        std::vector<double> next_amplitude(joint_names_.size(), 1.0);
        std::vector<double> next_offset(joint_names_.size(), 0.0);
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), joint_names_[i]);
            if (it == msg->joint_names.end()) return;
            size_t index = std::distance(msg->joint_names.begin(), it);
            if (!msg->amplitude_scale_valid[index] || !msg->offset_delta_valid[index] ||
                !std::isfinite(msg->amplitude_scale[index]) || msg->amplitude_scale[index] <= 0.0 ||
                !std::isfinite(msg->offset_delta[index]))
                return;
            next_amplitude[i] = msg->amplitude_scale[index];
            next_offset[i] = msg->offset_delta[index];
        }
        last_parameter_target_time_ = rclcpp::Time(msg->stamp);
        last_parameter_stamp_ = stamp;
        omega_modulation_.set(msg->omega_scale, received.seconds(), modulation_transition_s_);
        for (size_t j = 0; j < joint_names_.size(); ++j) {
            amplitude_modulation_[j].set(next_amplitude[j], received.seconds(), modulation_transition_s_);
            offset_modulation_[j].set(next_offset[j], received.seconds(), modulation_transition_s_);
        }
        continuous_modulation_active_ = true;
    }

    void update_modulation_status(const rclcpp::Time &now)
    {
        if (continuous_modulation_active_ &&
            (parameter_target_timeout_s_ <= 0.0 ||
             (now - last_parameter_target_time_).seconds() > parameter_target_timeout_s_))
        {
            reset_modulation(now.seconds());
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "CPG 参数目标超时，回退到当前 gait library 的已验证参数");
        }
    }

    void reset_modulation(double now)
    {
        continuous_modulation_active_ = false;
        omega_modulation_.set(1.0, now, modulation_transition_s_);
        for (size_t j = 0; j < joint_names_.size(); ++j) {
            amplitude_modulation_[j].set(1.0, now, modulation_transition_s_);
            offset_modulation_[j].set(0.0, now, modulation_transition_s_);
        }
    }

    std::string state_string() const
    {
        switch (cpg_state_)
        {
        case CPGState::IDLE: return "IDLE";
        case CPGState::STARTING: return "STARTING";
        case CPGState::WALKING: return "WALKING";
        case CPGState::STOPPING: return "STOPPING";
        case CPGState::SWITCHING: return "SWITCHING";
        }
        return "UNKNOWN";
    }

    void publish_structured_debug(const rclcpp::Time &stamp, double switch_progress, double omega,
        const std::vector<double> &w_offset, const std::vector<double> &w_x1,
        const std::vector<double> &w_x2, const std::vector<double> &w_x3,
        const std::vector<double> &w_y1, const std::vector<double> &w_y2,
        const std::vector<double> &w_y3, const std::vector<double> &joint_reference,
        const std::vector<double> &joint_velocity_reference)
    {
        terrain_adaptive_cpg::msg::CPGDebug msg;
        msg.header.stamp = stamp;
        msg.state = state_string();
        msg.gait_mode = current_gait_name_;
        msg.phase = std::atan2(y_, x_);
        msg.omega = omega;
        msg.switch_progress = switch_progress;
        msg.joint_names = joint_names_;
        if (w_offset.empty())
        {
            for (const auto &name : joint_names_)
            {
                const auto &model = current_gait_->joints.at(name);
                msg.w_offset.push_back(model.w_offset);
                auto coefficient = [](const std::vector<double> &values, size_t i) {
                    return i < values.size() ? values[i] : 0.0;
                };
                msg.w_x1.push_back(coefficient(model.w_x, 1));
                msg.w_x2.push_back(coefficient(model.w_x, 2));
                msg.w_x3.push_back(coefficient(model.w_x, 3));
                msg.w_y1.push_back(coefficient(model.w_y, 1));
                msg.w_y2.push_back(coefficient(model.w_y, 2));
                msg.w_y3.push_back(coefficient(model.w_y, 3));
                msg.joint_reference.push_back(0.0);
                msg.joint_velocity_reference.push_back(0.0);
            }
        }
        else
        {
            msg.w_offset = w_offset;
            msg.w_x1 = w_x1; msg.w_x2 = w_x2; msg.w_x3 = w_x3;
            msg.w_y1 = w_y1; msg.w_y2 = w_y2; msg.w_y3 = w_y3;
            msg.joint_reference = joint_reference;
            msg.joint_velocity_reference = joint_velocity_reference;
        }
        msg.continuous_modulation_active = continuous_modulation_active_;
        msg.amplitude_scale = amplitude_scales_;
        msg.offset_delta = offset_deltas_;
        // TODO(data_required): 只保留 phase-rate 接口，禁止强制相位重置或未标定增益。
        cpg_debug_pub_->publish(msg);
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
                amplitude_envelope_.set(1.0, ramp_start_time_.seconds(), ramp_time_);
                publish_state("STARTING");
                RCLCPP_INFO(this->get_logger(), "CPG 启动：开始渐入 (ramp_time=%.2fs)", ramp_time_);
            }
            else if (cpg_state_ == CPGState::STOPPING)
            {
                RCLCPP_WARN(this->get_logger(), "停止渐出期间忽略 start；进入 IDLE 后可重新启动");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "CPG 当前状态非 IDLE，忽略 start 指令");
            }
        }
        else if (cmd == "stop" || cmd == "STOP")
        {
            if (cpg_state_ == CPGState::WALKING || cpg_state_ == CPGState::STARTING || cpg_state_ == CPGState::SWITCHING)
            {
                cpg_state_ = CPGState::STOPPING;
                ramp_start_time_ = this->now();
                amplitude_envelope_.set(0.0, ramp_start_time_.seconds(), ramp_time_);
                publish_state("STOPPING");
                RCLCPP_INFO(this->get_logger(), "CPG 停止：开始渐出 (A=%.3f)", amplitude_factor_);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "CPG 当前状态非 WALKING/STARTING/SWITCHING，忽略 stop 指令");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "未知的 CPG 指令: '%s' (支持: start/stop)", cmd.c_str());
        }
    }

    // ==================== Gait Mode 切换回调 ====================
    void gait_mode_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string mode_name = msg->data;
        mode_name.erase(0, mode_name.find_first_not_of(" \t\n\r"));
        mode_name.erase(mode_name.find_last_not_of(" \t\n\r") + 1);

        if (gait_library_.find(mode_name) == gait_library_.end())
        {
            RCLCPP_WARN(this->get_logger(), "未知的 gait mode: '%s'，忽略", mode_name.c_str());
            return;
        }

        if (cpg_state_ == CPGState::STARTING || cpg_state_ == CPGState::SWITCHING ||
            cpg_state_ == CPGState::STOPPING) {
            RCLCPP_WARN(this->get_logger(), "启停或切换中忽略模式请求，完成后请重新提交");
            return;
        }

        // 如果目标 mode 与当前相同，忽略
        if (mode_name == current_gait_name_)
        {
            RCLCPP_WARN(this->get_logger(), "Gait mode 已经是 '%s'，忽略", mode_name.c_str());
            return;
        }

        // 如果当前是 IDLE 状态，直接切换（不需要过渡）
        if (cpg_state_ == CPGState::IDLE)
        {
            current_gait_name_ = mode_name;
            active_gait_ = gait_library_.at(current_gait_name_);
            current_gait_ = &active_gait_;
            omega_ = active_gait_.omega;
            switch_active_ = false;

            auto out = std_msgs::msg::String();
            out.data = current_gait_name_;
            gait_mode_pub_->publish(out);

            RCLCPP_INFO(this->get_logger(), "Gait mode 切换为: '%s' (omega=%.3f) [IDLE 直接切换]",
                        current_gait_name_.c_str(), current_gait_->omega);
            return;
        }

        // 仅在稳定行走状态开始平滑过渡
        if (cpg_state_ == CPGState::WALKING)
        {
            GaitMode *target_gait = &gait_library_[mode_name];

            // 保存新旧权重快照
            switch_old_weights_.clear();
            switch_new_weights_.clear();
            for (const auto &name : joint_names_)
            {
                // 旧权重（当前 gait mode）
                auto &old_m = current_gait_->joints[name];
                JointWeightSnapshot old_snap;
                old_snap.w_offset = old_m.w_offset;
                old_snap.w_x = old_m.w_x;
                old_snap.w_y = old_m.w_y;
                old_snap.limit_min = old_m.limit_min;
                old_snap.limit_max = old_m.limit_max;
                switch_old_weights_[name] = old_snap;

                // 新权重（目标 gait mode）
                auto &new_m = target_gait->joints[name];
                JointWeightSnapshot new_snap;
                new_snap.w_offset = new_m.w_offset;
                new_snap.w_x = new_m.w_x;
                new_snap.w_y = new_m.w_y;
                new_snap.limit_min = new_m.limit_min;
                new_snap.limit_max = new_m.limit_max;
                switch_new_weights_[name] = new_snap;
            }

            // 保存新旧 omega
            switch_old_omega_ = omega_;
            switch_new_omega_ = target_gait->omega;

            // 设置目标 gait mode
            switch_target_gait_ = mode_name;
            switch_target_gait_ptr_ = target_gait;

            // 开始过渡，保留连续振荡器状态
            switch_start_time_ = this->now();
            cpg_state_ = CPGState::SWITCHING;
            switch_active_ = true;
            publish_state("SWITCHING");

            RCLCPP_INFO(this->get_logger(), "Gait mode 平滑过渡开始: '%s' -> '%s' (omega: %.3f -> %.3f, 过渡时间=%.2fs)",
                        current_gait_name_.c_str(), switch_target_gait_.c_str(),
                        switch_old_omega_, switch_new_omega_, switch_duration_);
            return;
        }

        // 如果当前是 STOPPING 状态，忽略
        RCLCPP_WARN(this->get_logger(), "CPG 当前状态为 STOPPING，忽略 gait mode 切换指令");
    }

    Sample compute_switch_progress(const rclcpp::Time &now) const
    {
        return cpg_controller::progress((now - switch_start_time_).seconds(), switch_duration_);
    }

    // ==================== 发布状态 ====================
    void publish_state(const std::string &state_str)
    {
        auto msg = std_msgs::msg::String();
        msg.data = state_str;
        state_pub_->publish(msg);
    }

    // ==================== 参数更新 ====================
    void update_params()
    {
        mu_ = this->get_parameter("mu").as_double();
        gamma_ = this->get_parameter("gamma").as_double();
        eta_w_ = this->get_parameter("eta_w").as_double();
        eta_omega_ = this->get_parameter("eta_omega").as_double();
        ramp_time_ = this->get_parameter("ramp_time").as_double();
        publish_ankle_ = this->get_parameter("publish_ankle").as_bool();
        parameter_target_timeout_s_ = this->get_parameter("parameter_target_timeout_s").as_double();

        modulation_transition_s_ = get_parameter("modulation_transition_s").as_double();
        const double perturbation = get_parameter("initial_perturbation").as_double();
        if (!std::isfinite(mu_) || mu_ <= 0 || !std::isfinite(gamma_) || gamma_ <= 0 ||
            !std::isfinite(ramp_time_) || ramp_time_ < 0.01 ||
            !std::isfinite(parameter_target_timeout_s_) || parameter_target_timeout_s_ < 0 ||
            !std::isfinite(modulation_transition_s_) || modulation_transition_s_ < 0.01 ||
            !std::isfinite(perturbation) || perturbation <= 0 || perturbation > 1)
            throw std::runtime_error("Invalid CPG dynamics/timing parameters");
        if (eta_w_ != 0.0 || eta_omega_ != 0.0)
            throw std::runtime_error("Online learning is not implemented; eta_w/eta_omega must remain zero");
        omega_ = current_gait_->omega;
    }

    // ==================== 成员变量 ====================
    // 振荡器状态
    double x_, y_, omega_;
    double mu_, gamma_, eta_w_, eta_omega_;

    // 启停状态机
    CPGState cpg_state_;
    double amplitude_factor_;       // A(t) ∈ [0, 1]
    cpg_controller::Transition amplitude_envelope_;   // 启停包络及其时间导数
    double ramp_time_;              // 渐入/渐出时间常数
    rclcpp::Time ramp_start_time_;  // 当前渐变阶段的起始时刻

    bool switch_active_ = false;

    // Mode 切换过渡态
    std::string switch_target_gait_;           // 目标 gait mode 名称
    GaitMode *switch_target_gait_ptr_;         // 目标 gait mode 指针
    std::map<std::string, JointWeightSnapshot> switch_old_weights_;  // 切换前的权重快照
    std::map<std::string, JointWeightSnapshot> switch_new_weights_;  // 切换后的权重快照
    double switch_old_omega_;                  // 切换前的 omega
    double switch_new_omega_;                  // 切换后的 omega
    rclcpp::Time switch_start_time_;           // 切换开始时间
    double switch_duration_ = 0.5;             // 过渡时间（秒）

    // 踝关节开关
    bool publish_ankle_;
    bool continuous_modulation_active_ = false;
    double parameter_target_timeout_s_ = 0.0;
    double omega_scale_ = 1.0;
    std::vector<double> amplitude_scales_;
    std::vector<double> offset_deltas_;

    cpg_controller::PositiveTransition omega_modulation_;
    std::array<cpg_controller::PositiveTransition, 6> amplitude_modulation_{};
    std::array<cpg_controller::Transition, 6> offset_modulation_{};
    double modulation_transition_s_{0.5}, last_parameter_stamp_{0};
    cpg_controller::MeasurementInput measurements_;
    double measurement_timeout_s_{0.1};
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measurement_sub_;

    // Gait Library
    std::map<std::string, GaitMode> gait_library_;
    std::string current_gait_name_;
    GaitMode active_gait_;  // mutable runtime copy; library remains an immutable prior
    GaitMode *current_gait_;

    std::vector<std::string> joint_names_;
    std::vector<std::string> static_joint_names_;

    // ROS 2 基础设施
    rclcpp::TimerBase::SharedPtr cpg_timer_;      // CPG 积分定时器 (500Hz)
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cpg_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cpg_ref_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gait_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr switch_debug_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gait_mode_cmd_sub_;
    rclcpp::Publisher<terrain_adaptive_cpg::msg::CPGDebug>::SharedPtr cpg_debug_pub_;
    rclcpp::Subscription<terrain_adaptive_cpg::msg::CPGParameterTarget>::SharedPtr parameter_target_sub_;
    rclcpp::Time last_time_;
    rclcpp::Time last_parameter_target_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptiveCPGNode>());
    rclcpp::shutdown();
    return 0;
}
