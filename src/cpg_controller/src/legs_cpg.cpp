#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <terrain_adaptive_cpg/msg/cpg_debug.hpp>
#include <terrain_adaptive_cpg/msg/cpg_parameter_target.hpp>
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

// ==================== CPG 状态机枚举 ====================
// IDLE      : 静止态，输出全零（各关节在 w_offset 位置）
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
    double w_offset;         // w0 — 也是站立静止时的关节目标角
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
        this->declare_parameter("eta_w", 0.01);
        this->declare_parameter("eta_omega", 0.005);
        this->declare_parameter("ramp_time", 0.5);
        this->declare_parameter("initial_perturbation", 0.1);
        this->declare_parameter("publish_ankle", true);
        this->declare_parameter("parameter_target_timeout_s", 0.0);

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

            for (const auto &jname : joint_names_)
            {
                std::string jp = prefix + "joints." + jname + ".";
                this->declare_parameter(jp + "num_harmonics", 3);
                this->declare_parameter(jp + "limit_min", -1.5);
                this->declare_parameter(jp + "limit_max", 1.5);

                int n_h = this->get_parameter(jp + "num_harmonics").as_int();
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
        current_gait_ = &gait_library_[current_gait_name_];
        RCLCPP_INFO(this->get_logger(), "默认 gait mode: '%s'", current_gait_name_.c_str());

        // 3. ROS 话题
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cpg_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        cpg_ref_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/cpg/joint_ref", 10);

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
        stop_start_amplitude_ = 1.0;
        ramp_start_time_ = this->now();

        // 5. 初始化振荡器（IDLE 状态下冻结为零态）
        x_ = 0.0;
        y_ = 0.0;
        update_params();

        // 6. 创建定时器
        // CPG 积分定时器: 500Hz (2ms)
        cpg_timer_ = this->create_wall_timer(2ms,
            std::bind(&AdaptiveCPGNode::cpg_timer_callback, this));
        // 参数更新定时器: 100Hz (10ms)
        param_timer_ = this->create_wall_timer(10ms,
            std::bind(&AdaptiveCPGNode::param_timer_callback, this));
        param_timer_active_ = true;

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
        double switch_alpha = 1.0;
        double omega_current = omega_;
        if (cpg_state_ == CPGState::SWITCHING)
        {
            switch_alpha = compute_switch_progress();
            omega_current = (1.0 - switch_alpha) * switch_old_omega_ + switch_alpha * switch_new_omega_;
        }
        if (continuous_modulation_active_)
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

        // --- 5. 计算谐波时间导数（用于关节速度输出） ---
        {
            double r2_final = x_ * x_ + y_ * y_;
            double dx_final = gamma_ * (mu_ - r2_final) * x_ - omega_current * y_;
            double dy_final = gamma_ * (mu_ - r2_final) * y_ + omega_current * x_;

            dhx_[1] = dx_final;
            dhy_[1] = dy_final;

            if (max_h_ >= 2)
            {
                dhx_[2] = 2.0 * (x_ * dx_final - y_ * dy_final);
                dhy_[2] = 2.0 * (dx_final * y_ + x_ * dy_final);
            }

            if (max_h_ >= 3)
            {
                double x2_y2 = x_ * x_ - y_ * y_;
                dhx_[3] = 3.0 * dx_final * x2_y2 - 6.0 * x_ * y_ * dy_final;
                dhy_[3] = 6.0 * x_ * y_ * dx_final + 3.0 * dy_final * x2_y2;
            }
        }

        // --- 6. 计算振幅因子 A(t) 和其导数 dA/dt ---
        amplitude_factor_ = compute_amplitude_factor();
        double d_amplitude_dt = compute_d_amplitude_dt(dt);

        // --- 6. 状态转换检测 ---
        if (cpg_state_ == CPGState::STARTING && amplitude_factor_ >= 0.999)
        {
            amplitude_factor_ = 1.0;
            cpg_state_ = CPGState::WALKING;
            publish_state("WALKING");
            // 启动参数更新定时器
            activate_param_timer();
            RCLCPP_INFO(this->get_logger(), "CPG 已完全启动，进入 WALKING 状态");
        }
        else if (cpg_state_ == CPGState::STOPPING && amplitude_factor_ <= 0.001)
        {
            amplitude_factor_ = 0.0;
            cpg_state_ = CPGState::IDLE;
            x_ = 0.0;
            y_ = 0.0;
            // 停止参数更新定时器
            deactivate_param_timer();
            publish_state("IDLE");
            RCLCPP_INFO(this->get_logger(), "CPG 已完全停止，进入 IDLE 状态");
        }

        // --- 7. 高阶谐波生成 ---
        hx_[1] = x_;
        hy_[1] = y_;

        if (max_h_ >= 2)
        {
            hx_[2] = x_ * x_ - y_ * y_;
            hy_[2] = 2.0 * x_ * y_;
        }

        if (max_h_ >= 3)
        {
            hx_[3] = x_ * (x_ * x_ - 3.0 * y_ * y_);
            hy_[3] = y_ * (3.0 * x_ * x_ - y_ * y_);
        }

        // --- 8. 切换完成检测（switch_alpha 已在 step 3 计算） ---
        if (cpg_state_ == CPGState::SWITCHING && switch_alpha >= 1.0)
        {
            current_gait_name_ = switch_target_gait_;
            current_gait_ = switch_target_gait_ptr_;
            omega_ = switch_new_omega_;

            auto out = std_msgs::msg::String();
            out.data = current_gait_name_;
            gait_mode_pub_->publish(out);

            cpg_state_ = CPGState::WALKING;
            publish_state("WALKING");

            // 切换完成，重新启动参数更新定时器
            activate_param_timer();

            RCLCPP_INFO(this->get_logger(), "Gait mode 平滑过渡完成: '%s' (omega=%.3f)",
                        current_gait_name_.c_str(), omega_);
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

            // 插值权重
            double w_off_cur = m.w_offset;
            double wx1 = 0.0, wx2 = 0.0, wx3 = 0.0;
            double wy1 = 0.0, wy2 = 0.0, wy3 = 0.0;

            if (cpg_state_ == CPGState::SWITCHING && switch_old_weights_.count(name))
            {
                auto &old_w = switch_old_weights_[name];
                auto &new_w = switch_new_weights_[name];
                double a = switch_alpha;

                auto interp = [&](const std::vector<double> &ov, const std::vector<double> &nv, int i) -> double {
                    double o = (i < (int)ov.size()) ? ov[i] : 0.0;
                    double n = (i < (int)nv.size()) ? nv[i] : 0.0;
                    return (1.0 - a) * o + a * n;
                };

                w_off_cur = (1.0 - a) * old_w.w_offset + a * new_w.w_offset;
                wx1 = interp(old_w.w_x, new_w.w_x, 1);
                wx2 = interp(old_w.w_x, new_w.w_x, 2);
                wx3 = interp(old_w.w_x, new_w.w_x, 3);
                wy1 = interp(old_w.w_y, new_w.w_y, 1);
                wy2 = interp(old_w.w_y, new_w.w_y, 2);
                wy3 = interp(old_w.w_y, new_w.w_y, 3);
            }
            else
            {
                if (1 < (int)m.w_x.size()) { wx1 = m.w_x[1]; }
                if (2 < (int)m.w_x.size()) { wx2 = m.w_x[2]; }
                if (3 < (int)m.w_x.size()) { wx3 = m.w_x[3]; }
                if (1 < (int)m.w_y.size()) { wy1 = m.w_y[1]; }
                if (2 < (int)m.w_y.size()) { wy2 = m.w_y[2]; }
                if (3 < (int)m.w_y.size()) { wy3 = m.w_y[3]; }
            }

            const size_t joint_index = dbg_w_off.size();
            if (continuous_modulation_active_)
            {
                w_off_cur += offset_deltas_[joint_index];
                wx1 *= amplitude_scales_[joint_index];
                wx2 *= amplitude_scales_[joint_index];
                wx3 *= amplitude_scales_[joint_index];
                wy1 *= amplitude_scales_[joint_index];
                wy2 *= amplitude_scales_[joint_index];
                wy3 *= amplitude_scales_[joint_index];
            }

            // 计算 theta（clamp 前）
            double theta_interp = w_off_cur;
            theta_interp += wx1 * hx_[1] + wy1 * hy_[1];
            theta_interp += wx2 * hx_[2] + wy2 * hy_[2];
            theta_interp += wx3 * hx_[3] + wy3 * hy_[3];

            // 振幅调制
            double output = amplitude_factor_ * theta_interp;

            // 计算关节速度: d(A*theta)/dt = A * dtheta/dt + theta * dA/dt
            double dtheta = wx1 * dhx_[1] + wy1 * dhy_[1];
            if (max_h_ >= 2) dtheta += wx2 * dhx_[2] + wy2 * dhy_[2];
            if (max_h_ >= 3) dtheta += wx3 * dhx_[3] + wy3 * dhy_[3];
            double velocity = amplitude_factor_ * dtheta + d_amplitude_dt * theta_interp;
            joint_velocities.push_back(velocity);

            // 限位
            double lim_min = m.limit_min;
            double lim_max = m.limit_max;
            if (cpg_state_ == CPGState::SWITCHING && switch_old_weights_.count(name))
            {
                lim_min = std::min(switch_old_weights_[name].limit_min, switch_new_weights_[name].limit_min);
                lim_max = std::max(switch_old_weights_[name].limit_max, switch_new_weights_[name].limit_max);
            }
            output = std::clamp(output, lim_min, lim_max);

            bool is_ankle = (name.find("ankle") != std::string::npos);
            double cmd_output = output;
            if (is_ankle && !publish_ankle_)
                cmd_output = 0.0;

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
        last_parameter_target_time_ = this->now();
        continuous_modulation_active_ = false;
        if (!msg->continuous_modulation_enabled || parameter_target_timeout_s_ <= 0.0)
            return;
        if (!msg->omega_scale_valid || !std::isfinite(msg->omega_scale) || msg->omega_scale <= 0.0)
            return;
        if (msg->joint_names.size() != msg->amplitude_scale.size() ||
            msg->joint_names.size() != msg->amplitude_scale_valid.size() ||
            msg->joint_names.size() != msg->offset_delta.size() ||
            msg->joint_names.size() != msg->offset_delta_valid.size())
            return;

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
        omega_scale_ = msg->omega_scale;
        amplitude_scales_ = std::move(next_amplitude);
        offset_deltas_ = std::move(next_offset);
        continuous_modulation_active_ = true;
    }

    void update_modulation_status(const rclcpp::Time &now)
    {
        if (continuous_modulation_active_ &&
            (parameter_target_timeout_s_ <= 0.0 ||
             (now - last_parameter_target_time_).seconds() > parameter_target_timeout_s_))
        {
            continuous_modulation_active_ = false;
            omega_scale_ = 1.0;
            std::fill(amplitude_scales_.begin(), amplitude_scales_.end(), 1.0);
            std::fill(offset_deltas_.begin(), offset_deltas_.end(), 0.0);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "CPG 参数目标超时，回退到当前 gait library 的已验证参数");
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

    // ==================== 参数更新定时器回调 (100Hz) ====================
    void param_timer_callback()
    {
        if (!param_timer_active_) return;

        double dt = 0.01;  // 100Hz = 10ms

        for (const auto &name : joint_names_)
        {
            auto &m = current_gait_->joints[name];

            // 计算当前输出
            double theta_interp = m.w_offset;
            for (int i = 1; i < m.num_harmonics; ++i)
            {
                theta_interp += m.w_x[i] * hx_[i] + m.w_y[i] * hy_[i];
            }

            // 学习（eta_w=0 时不影响输出）
            double theta_target = get_target_from_tf(name, theta_interp);
            double error = theta_target - theta_interp;
            m.w_offset += eta_w_ * error * dt;
            for (int i = 1; i < m.num_harmonics; ++i)
            {
                m.w_x[i] += eta_w_ * error * hx_[i] * dt;
                m.w_y[i] += eta_w_ * error * hy_[i] * dt;
            }

            if (name == "left_hip3")
                omega_ += eta_omega_ * error * hy_[1] * dt;
        }
    }

    // ==================== 参数更新定时器控制 ====================
    void activate_param_timer()
    {
        param_timer_active_ = true;
        RCLCPP_DEBUG(this->get_logger(), "参数更新定时器已激活");
    }

    void deactivate_param_timer()
    {
        param_timer_active_ = false;
        RCLCPP_DEBUG(this->get_logger(), "参数更新定时器已停止");
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
            if (cpg_state_ == CPGState::WALKING || cpg_state_ == CPGState::STARTING || cpg_state_ == CPGState::SWITCHING)
            {
                cpg_state_ = CPGState::STOPPING;
                stop_start_amplitude_ = amplitude_factor_;
                ramp_start_time_ = this->now();
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
            current_gait_ = &gait_library_[current_gait_name_];

            auto out = std_msgs::msg::String();
            out.data = current_gait_name_;
            gait_mode_pub_->publish(out);

            RCLCPP_INFO(this->get_logger(), "Gait mode 切换为: '%s' (omega=%.3f) [IDLE 直接切换]",
                        current_gait_name_.c_str(), current_gait_->omega);
            return;
        }

        // 如果当前是 WALKING、STARTING 或 SWITCHING 状态，开始平滑过渡
        if (cpg_state_ == CPGState::WALKING || cpg_state_ == CPGState::STARTING || cpg_state_ == CPGState::SWITCHING)
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

            // 开始过渡，停止参数更新
            switch_start_time_ = this->now();
            cpg_state_ = CPGState::SWITCHING;
            deactivate_param_timer();
            publish_state("SWITCHING");

            RCLCPP_INFO(this->get_logger(), "Gait mode 平滑过渡开始: '%s' -> '%s' (omega: %.3f -> %.3f, 过渡时间=%.2fs)",
                        current_gait_name_.c_str(), switch_target_gait_.c_str(),
                        switch_old_omega_, switch_new_omega_, switch_duration_);
            return;
        }

        // 如果当前是 STOPPING 状态，忽略
        RCLCPP_WARN(this->get_logger(), "CPG 当前状态为 STOPPING，忽略 gait mode 切换指令");
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

        case CPGState::SWITCHING:
            // 切换过程中保持全振幅，输出插值在 on_timer 中处理
            return 1.0;

        default:
            return 0.0;
        }
    }

    // ==================== 振幅因子导数 dA/dt ====================
    double compute_d_amplitude_dt(double /*dt*/)
    {
        double elapsed = (this->now() - ramp_start_time_).seconds();
        double T = ramp_time_;
        if (T < 0.01) T = 0.01;

        switch (cpg_state_)
        {
        case CPGState::IDLE:
            return 0.0;

        case CPGState::STARTING:
        {
            double t = std::clamp(elapsed / T, 0.0, 1.0);
            // dA/dt = 0.5 * (π/T) * sin(π*t/T)
            return 0.5 * (M_PI / T) * std::sin(M_PI * t);
        }

        case CPGState::WALKING:
            return 0.0;

        case CPGState::STOPPING:
        {
            double t = std::clamp(elapsed / T, 0.0, 1.0);
            // dA/dt = -stop_start * 0.5 * (π/T) * sin(π*t/T)
            return -stop_start_amplitude_ * 0.5 * (M_PI / T) * std::sin(M_PI * t);
        }

        case CPGState::SWITCHING:
            return 0.0;

        default:
            return 0.0;
        }
    }

    // ==================== 计算切换过渡进度 ====================
    // 返回 0.0 ~ 1.0 的平滑过渡因子
    // 使用五次多项式 6t^5 - 15t^4 + 10t^3，C2 连续（位置、速度、加速度都连续）
    double compute_switch_progress()
    {
        double elapsed = (this->now() - switch_start_time_).seconds();
        double t = std::clamp(elapsed / switch_duration_, 0.0, 1.0);
        // 五次多项式: s(t) = 6t^5 - 15t^4 + 10t^3
        // s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0
        return t * t * t * (t * (6.0 * t - 15.0) + 10.0);
    }

    // ==================== 发布状态 ====================
    void publish_state(const std::string &state_str)
    {
        auto msg = std_msgs::msg::String();
        msg.data = state_str;
        state_pub_->publish(msg);
    }

    // ==================== 从 TF 获取目标角度 ====================
    double get_target_from_tf(const std::string & /*joint_name*/, double current_cpg)
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
        eta_w_ = this->get_parameter("eta_w").as_double();
        eta_omega_ = this->get_parameter("eta_omega").as_double();
        ramp_time_ = this->get_parameter("ramp_time").as_double();
        publish_ankle_ = this->get_parameter("publish_ankle").as_bool();
        parameter_target_timeout_s_ = this->get_parameter("parameter_target_timeout_s").as_double();

        // omega 和 max_h_ 从当前 gait mode 读取
        omega_ = current_gait_->omega;
        max_h_ = 1;
        for (auto &pair : current_gait_->joints)
            max_h_ = std::max(max_h_, pair.second.num_harmonics - 1);
    }

    // ==================== 成员变量 ====================
    // 振荡器状态
    double x_, y_, omega_;
    double mu_, gamma_, eta_w_, eta_omega_;
    int max_h_;

    // 谐波特征（成员变量，供切换过渡时使用）
    std::vector<double> hx_{5, 0.0}, hy_{5, 0.0};
    std::vector<double> dhx_{5, 0.0}, dhy_{5, 0.0};  // 谐波时间导数

    // 启停状态机
    CPGState cpg_state_;
    double amplitude_factor_;       // A(t) ∈ [0, 1]
    double stop_start_amplitude_;   // 进入 STOPPING 瞬间的 A 值
    double ramp_time_;              // 渐入/渐出时间常数
    rclcpp::Time ramp_start_time_;  // 当前渐变阶段的起始时刻

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

    // Gait Library
    std::map<std::string, GaitMode> gait_library_;
    std::string current_gait_name_;
    GaitMode *current_gait_;

    std::vector<std::string> joint_names_;
    std::vector<std::string> static_joint_names_;

    // ROS 2 基础设施
    rclcpp::TimerBase::SharedPtr cpg_timer_;      // CPG 积分定时器 (500Hz)
    rclcpp::TimerBase::SharedPtr param_timer_;    // 参数更新定时器 (100Hz)
    bool param_timer_active_;                     // 参数更新定时器是否激活
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
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
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
