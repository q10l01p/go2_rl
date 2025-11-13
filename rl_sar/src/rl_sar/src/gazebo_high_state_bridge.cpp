#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/bind/bind.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unitree_legged_msgs/HighState.h>

using boost::placeholders::_1;

namespace {

constexpr std::array<const char*, 4> kLegPrefixes = {"FL", "FR", "RL", "RR"};
constexpr std::array<const char*, 3> kJointSuffixes = {"hip", "thigh", "calf"};

inline int16_t saturateToInt16(double value) {
    const double rounded = std::round(value);
    if (rounded > static_cast<double>(std::numeric_limits<int16_t>::max())) {
        return std::numeric_limits<int16_t>::max();
    }
    if (rounded < static_cast<double>(std::numeric_limits<int16_t>::min())) {
        return std::numeric_limits<int16_t>::min();
    }
    return static_cast<int16_t>(rounded);
}

}  // namespace

class GazeboHighStateBridge {
   public:
    GazeboHighStateBridge(ros::NodeHandle nh, ros::NodeHandle pnh)
        : nh_(std::move(nh)),
          pnh_(std::move(pnh)),
          publish_rate_hz_(200.0),
          foot_force_scale_(1.0),
          foot_force_bias_(0.0),
          foot_force_use_abs_(true) {
        this->loadParameters();
        this->initRosEntities();
    }

    void spin() const { ros::spin(); }

   private:
    void loadParameters() {
        pnh_.param<std::string>("high_state_topic", high_state_topic_, "/high_state");
        pnh_.param<std::string>("imu_topic", imu_topic_, "/trunk_imu");
        pnh_.param<std::string>("joint_state_topic", joint_state_topic_, "/go2_gazebo/joint_states");

        for (size_t i = 0; i < foot_force_topics_.size(); ++i) {
            const std::string param_name = std::string("foot_force_topic_") + kLegPrefixes[i];
            const std::string default_topic =
                std::string("/visual/") + kLegPrefixes[i] + "_foot_contact/the_force";
            pnh_.param<std::string>(param_name, foot_force_topics_[i], default_topic);
        }

        pnh_.param("publish_rate_hz", publish_rate_hz_, publish_rate_hz_);
        pnh_.param("foot_force_scale", foot_force_scale_, foot_force_scale_);
        pnh_.param("foot_force_bias", foot_force_bias_, foot_force_bias_);
        pnh_.param("foot_force_use_abs", foot_force_use_abs_, foot_force_use_abs_);
    }

    void initRosEntities() {
        ros::TransportHints hints = ros::TransportHints().tcpNoDelay(true);
        imu_sub_ = nh_.subscribe(imu_topic_, 10, &GazeboHighStateBridge::imuCallback, this, hints);
        joint_state_sub_ =
            nh_.subscribe(joint_state_topic_, 10, &GazeboHighStateBridge::jointCallback, this, hints);
        for (size_t i = 0; i < foot_force_topics_.size(); ++i) {
            const auto callback =
                boost::bind(&GazeboHighStateBridge::footForceCallback, this, _1, static_cast<int>(i));
            foot_force_subs_[i] = nh_.subscribe<geometry_msgs::WrenchStamped>(
                foot_force_topics_[i], 10, callback, ros::VoidConstPtr(), hints);
        }

        high_state_pub_ = nh_.advertise<unitree_legged_msgs::HighState>(high_state_topic_, 10);
        publish_timer_ =
            nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_hz_)),
                            &GazeboHighStateBridge::publishTimerCallback, this);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_imu_ = *msg;
        imu_received_ = true;
    }

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_joint_state_ = *msg;
        joints_received_ = ensureJointMapping(latest_joint_state_);
    }

    void footForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg, int index) {
        if (index < 0 || index >= static_cast<int>(foot_force_topics_.size())) { return; }

        double force_value = msg->wrench.force.z;
        if (foot_force_use_abs_) { force_value = std::abs(force_value); }
        force_value = force_value * foot_force_scale_ + foot_force_bias_;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            foot_forces_[index] = force_value;
            foot_force_stamps_[index] = msg->header.stamp;
        }
    }

    void publishTimerCallback(const ros::TimerEvent&) {
        unitree_legged_msgs::HighState high_state_msg;
        sensor_msgs::Imu imu_msg;
        sensor_msgs::JointState joint_msg;
        std::array<double, 4> foot_forces{};
        std::array<ros::Time, 4> foot_stamps{};

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!imu_received_ || !joints_received_) {
                ROS_WARN_THROTTLE(5.0, "[gazebo_high_state_bridge] Waiting for IMU and joint data.");
                return;
            }

            imu_msg = latest_imu_;
            joint_msg = latest_joint_state_;
            foot_forces = foot_forces_;
            foot_stamps = foot_force_stamps_;
        }

        populateHighStateMessage(imu_msg, joint_msg, foot_forces, foot_stamps, high_state_msg);
        high_state_pub_.publish(high_state_msg);
    }

    bool ensureJointMapping(const sensor_msgs::JointState& msg) {
        if (joint_mapping_ready_) { return true; }
        if (msg.name.empty()) {
            ROS_WARN_THROTTLE(5.0, "[gazebo_high_state_bridge] Received joint state without names.");
            return false;
        }

        std::unordered_map<std::string, size_t> name_to_index;
        name_to_index.reserve(msg.name.size());
        for (size_t idx = 0; idx < msg.name.size(); ++idx) { name_to_index[msg.name[idx]] = idx; }

        for (size_t leg = 0; leg < kLegPrefixes.size(); ++leg) {
            for (size_t joint = 0; joint < kJointSuffixes.size(); ++joint) {
                const std::string joint_name =
                    std::string(kLegPrefixes[leg]) + "_" + kJointSuffixes[joint] + "_joint";
                auto iter = name_to_index.find(joint_name);
                if (iter == name_to_index.end()) {
                    ROS_WARN_THROTTLE(5.0, "[gazebo_high_state_bridge] Joint %s not found in joint state.",
                                      joint_name.c_str());
                    return false;
                }
                joint_name_to_index_[leg * 3 + joint] = iter->second;
            }
        }

        joint_mapping_ready_ = true;
        ROS_INFO("[gazebo_high_state_bridge] Joint mapping initialised.");
        return true;
    }

    void populateHighStateMessage(const sensor_msgs::Imu& imu_msg, const sensor_msgs::JointState& joint_msg,
                                  const std::array<double, 4>& foot_forces,
                                  const std::array<ros::Time, 4>& foot_stamps,
                                  unitree_legged_msgs::HighState& high_state_msg) const {
        high_state_msg.stamp = selectTimestamp(imu_msg.header.stamp, joint_msg.header.stamp, foot_stamps);

        fillImu(imu_msg, high_state_msg);
        fillMotorStates(joint_msg, high_state_msg);
        fillFootForces(foot_forces, high_state_msg);
        fillPlaceholders(high_state_msg);
    }

    static ros::Time selectTimestamp(const ros::Time& imu_stamp, const ros::Time& joint_stamp,
                                     const std::array<ros::Time, 4>& foot_stamps) {
        ros::Time selected = imu_stamp;
        if (!joint_stamp.isZero() && (selected.isZero() || joint_stamp > selected)) { selected = joint_stamp; }
        for (const auto& stamp : foot_stamps) {
            if (!stamp.isZero() && (selected.isZero() || stamp > selected)) { selected = stamp; }
        }
        if (selected.isZero()) { selected = ros::Time::now(); }
        return selected;
    }

    static void fillImu(const sensor_msgs::Imu& imu_msg, unitree_legged_msgs::HighState& high_state_msg) {
        auto& imu = high_state_msg.imu;
        imu.quaternion[0] = static_cast<float>(imu_msg.orientation.w);
        imu.quaternion[1] = static_cast<float>(imu_msg.orientation.x);
        imu.quaternion[2] = static_cast<float>(imu_msg.orientation.y);
        imu.quaternion[3] = static_cast<float>(imu_msg.orientation.z);

        imu.gyroscope[0] = static_cast<float>(imu_msg.angular_velocity.x);
        imu.gyroscope[1] = static_cast<float>(imu_msg.angular_velocity.y);
        imu.gyroscope[2] = static_cast<float>(imu_msg.angular_velocity.z);

        imu.accelerometer[0] = static_cast<float>(imu_msg.linear_acceleration.x);
        imu.accelerometer[1] = static_cast<float>(imu_msg.linear_acceleration.y);
        imu.accelerometer[2] = static_cast<float>(imu_msg.linear_acceleration.z);

        tf2::Quaternion quat(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z,
                             imu_msg.orientation.w);
        tf2::Matrix3x3 matrix(quat);
        double roll{0.0}, pitch{0.0}, yaw{0.0};
        matrix.getRPY(roll, pitch, yaw);
        imu.rpy[0] = static_cast<float>(roll);
        imu.rpy[1] = static_cast<float>(pitch);
        imu.rpy[2] = static_cast<float>(yaw);
        imu.temperature = 0;
    }

    void fillMotorStates(const sensor_msgs::JointState& joint_msg,
                         unitree_legged_msgs::HighState& high_state_msg) const {
        auto& motor_states = high_state_msg.motorState;
        const auto positions_size = joint_msg.position.size();
        const auto velocities_size = joint_msg.velocity.size();

        for (size_t i = 0; i < motor_states.size(); ++i) {
            motor_states[i].mode = 0;
            motor_states[i].q = 0.F;
            motor_states[i].dq = 0.F;
            motor_states[i].ddq = 0.F;
            motor_states[i].tauEst = 0.F;
            motor_states[i].q_raw = 0.F;
            motor_states[i].dq_raw = 0.F;
            motor_states[i].ddq_raw = 0.F;
            motor_states[i].temperature = 0;
            motor_states[i].reserve[0] = 0;
            motor_states[i].reserve[1] = 0;
        }

        for (size_t leg = 0; leg < kLegPrefixes.size(); ++leg) {
            for (size_t joint = 0; joint < kJointSuffixes.size(); ++joint) {
                const size_t motor_index = leg * 3 + joint;
                if (motor_index >= motor_states.size()) { continue; }
                const size_t joint_state_index = joint_name_to_index_[motor_index];
                if (joint_state_index >= positions_size) { continue; }

                const double position = joint_msg.position[joint_state_index];
                const double velocity =
                    (joint_state_index < velocities_size) ? joint_msg.velocity[joint_state_index] : 0.0;

                motor_states[motor_index].q = static_cast<float>(position);
                motor_states[motor_index].dq = static_cast<float>(velocity);
                motor_states[motor_index].q_raw = static_cast<float>(position);
                motor_states[motor_index].dq_raw = static_cast<float>(velocity);
            }
        }
    }

    void fillFootForces(const std::array<double, 4>& foot_forces,
                        unitree_legged_msgs::HighState& high_state_msg) const {
        for (size_t i = 0; i < foot_forces.size(); ++i) {
            const int16_t force_value = saturateToInt16(foot_forces[i]);
            high_state_msg.footForce[i] = force_value;
            high_state_msg.footForceEst[i] = force_value;
        }
    }

    static void fillPlaceholders(unitree_legged_msgs::HighState& high_state_msg) {
        high_state_msg.levelFlag = 0;
        high_state_msg.frameReserve = 0;
        high_state_msg.bandWidth = 0;
        high_state_msg.mode = 0;
        high_state_msg.progress = 0.F;
        high_state_msg.gaitType = 0;
        high_state_msg.footRaiseHeight = 0.F;
        high_state_msg.bodyHeight = 0.F;
        high_state_msg.yawSpeed = 0.F;
        high_state_msg.reserve = 0;
        high_state_msg.crc = 0;

        for (auto& value : high_state_msg.head) { value = 0; }
        for (auto& value : high_state_msg.SN) { value = 0; }
        for (auto& value : high_state_msg.version) { value = 0; }
        for (auto& value : high_state_msg.velocity) { value = 0.F; }
        for (auto& value : high_state_msg.position) { value = 0.F; }
        for (auto& value : high_state_msg.rangeObstacle) { value = 0.F; }
        for (auto& value : high_state_msg.wirelessRemote) { value = 0; }

        auto& bms = high_state_msg.bms;
        bms.version_h = 0;
        bms.version_l = 0;
        bms.bms_status = 0;
        bms.SOC = 0;
        bms.current = 0;
        bms.cycle = 0;
        for (auto& val : bms.BQ_NTC) { val = 0; }
        for (auto& val : bms.MCU_NTC) { val = 0; }
        for (auto& val : bms.cell_vol) { val = 0; }

        for (auto& cartesian : high_state_msg.footPosition2Body) {
            cartesian.x = 0.F;
            cartesian.y = 0.F;
            cartesian.z = 0.F;
        }
        for (auto& cartesian : high_state_msg.footSpeed2Body) {
            cartesian.x = 0.F;
            cartesian.y = 0.F;
            cartesian.z = 0.F;
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string high_state_topic_;
    std::string imu_topic_;
    std::string joint_state_topic_;
    std::array<std::string, 4> foot_force_topics_{};

    ros::Subscriber imu_sub_;
    ros::Subscriber joint_state_sub_;
    std::array<ros::Subscriber, 4> foot_force_subs_{};
    ros::Publisher high_state_pub_;
    ros::Timer publish_timer_;

    double publish_rate_hz_;
    double foot_force_scale_;
    double foot_force_bias_;
    bool foot_force_use_abs_;

    mutable std::mutex data_mutex_;
    sensor_msgs::Imu latest_imu_;
    sensor_msgs::JointState latest_joint_state_;
    std::array<double, 4> foot_forces_{{0.0, 0.0, 0.0, 0.0}};
    std::array<ros::Time, 4> foot_force_stamps_{};
    bool imu_received_{false};
    bool joints_received_{false};
    std::array<size_t, 12> joint_name_to_index_{};
    bool joint_mapping_ready_{false};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_high_state_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    GazeboHighStateBridge bridge(nh, pnh);
    bridge.spin();
    return 0;
}
