#include "rover_mavlink_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>

RoverMavlinkNode::RoverMavlinkNode()
    : Node("rover_mavlink_node"),
      last_command_time_(0, 0, RCL_ROS_TIME),
      last_rf2o_time_(0, 0, RCL_ROS_TIME),
      state_deadline_(0, 0, RCL_ROS_TIME),
      arm_attempts_(0),
      autonomous_enabled_(false),
      command_received_(false),
      heartbeat_triggered_(false),
    //   estimator_status_received_(false),
    //   ekf_valid_(false),
      rf2o_received_(false),
      external_nav_selected_(false),
      autonomous_state_(AutonomousState::IDLE)
{
    max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 1.0);
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 1.0);
    heartbeat_timeout_ = declare_parameter<double>("heartbeat_timeout", 0.5);
    rf2o_timeout_ = declare_parameter<double>("rf2o_timeout", 0.5);
    guided_timeout_ = declare_parameter<double>("guided_timeout", 2.0);
    ekf_timeout_ = declare_parameter<double>("ekf_timeout", 3.0);
    arm_timeout_ = declare_parameter<double>("arm_timeout", 1.0);
    max_arm_attempts_ = declare_parameter<int>("max_arm_attempts", 5);
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    autonomous_enabled_ = declare_parameter<bool>("autonomous_enabled", false);
    forward_rf2o_to_mavros_ =
        declare_parameter<bool>("forward_rf2o_to_mavros", true);
    const bool enable_external_pose_publisher =
        declare_parameter<bool>("enable_external_pose_publisher", false);
    rf2o_topic_ = declare_parameter<std::string>("rf2o_topic", "/odom_rf2o");
    mavros_odometry_topic_ = declare_parameter<std::string>(
        "mavros_odometry_topic", "/mavros/odometry/out");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    target_odom_rate_hz_ = declare_parameter<double>("target_odom_rate_hz", 6.0);

    this->declare_parameter<double>(
    "local_position_timeout", 1.0);


    local_position_timeout_ =
    this->get_parameter(
        "local_position_timeout").as_double();


    local_position_subscriber_ =
    this->create_subscription<
        geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose",
        rclcpp::SensorDataQoS(),
        std::bind(
            &RoverMavlinkNode::localPositionCallback,
            this,
            std::placeholders::_1));


    if (forward_rf2o_to_mavros_ && rf2o_topic_ == mavros_odometry_topic_)
    {
        throw std::invalid_argument(
            "rf2o_topic and mavros_odometry_topic must differ when forwarding is enabled");
    }

    ardupilot_ = std::make_shared<ArduPilotInterface>(this);

    if (enable_external_pose_publisher && forward_rf2o_to_mavros_)
    {
        RCLCPP_WARN(
            get_logger(),
            "Both external pose and RF2O forwarding were requested. "
            "External pose publisher stays disabled to avoid duplicate ExternalNav inputs.");
    }
    else if (enable_external_pose_publisher)
    {
        external_pose_publisher_ = std::make_shared<ExternalPosePublisher>(this);
    }

    velocity_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&RoverMavlinkNode::velocityCallback, this, std::placeholders::_1));

    autonomous_subscriber_ = create_subscription<std_msgs::msg::Bool>(
        "/autonomous_enabled",
        10,
        std::bind(&RoverMavlinkNode::autonomousCallback, this, std::placeholders::_1));

    // estimator_status_subscriber_ =
    //     create_subscription<mavros_msgs::msg::EstimatorStatus>(
    //     "/mavros/estimator_status",
    //     10,
    //     std::bind(
    //         &RoverMavlinkNode::estimatorStatusCallback,
    //         this,
    //         std::placeholders::_1));

    rf2o_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
        rf2o_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&RoverMavlinkNode::rf2oCallback, this, std::placeholders::_1));

    if (forward_rf2o_to_mavros_)
    {
        mavros_odometry_publisher_ =
            create_publisher<nav_msgs::msg::Odometry>(mavros_odometry_topic_, 1);
    }

    heartbeat_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RoverMavlinkNode::heartbeatCallback, this));

    autonomous_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RoverMavlinkNode::autonomousStateCallback, this));

    RCLCPP_INFO(get_logger(), "Rover MAVLink communication node started");
    RCLCPP_INFO(
        get_logger(),
        "Autonomous request=%s, auto_arm=%s, RF2O=%s",
        autonomous_enabled_ ? "true" : "false",
        auto_arm_ ? "true" : "false",
        rf2o_topic_.c_str());
}

void RoverMavlinkNode::velocityCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    last_command_time_ = now();
    command_received_ = true;

    if (heartbeat_triggered_)
    {
        RCLCPP_INFO(get_logger(), "Navigation command stream restored");
        heartbeat_triggered_ = false;
    }

    if (!isAutonomousRequested() || autonomous_state_ != AutonomousState::ACTIVE)
    {
        return;
    }

    const double linear_velocity = std::clamp(
        msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
    const double angular_velocity = std::clamp(
        msg->angular.z, -max_angular_velocity_, max_angular_velocity_);

    ardupilot_->setVelocity(linear_velocity, angular_velocity);

    RCLCPP_DEBUG(
        get_logger(),
        "Velocity command: %.2f m/s, yaw rate: %.2f rad/s",
        linear_velocity,
        angular_velocity);
}

void RoverMavlinkNode::autonomousCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (autonomous_enabled_ == msg->data)
    {
        return;
    }

    autonomous_enabled_ = msg->data;
    arm_attempts_ = 0;
    heartbeat_triggered_ = false;

    RCLCPP_INFO(
        get_logger(),
        "Autonomous operation %s",
        autonomous_enabled_ ? "requested" : "disabled");

    if (autonomous_enabled_)
    {
        autonomous_state_ = AutonomousState::IDLE;
    }
    else
    {
        ardupilot_->stop();
        autonomous_state_ = AutonomousState::REQUEST_MANUAL;
    }
}

// void RoverMavlinkNode::estimatorStatusCallback(
//     const mavros_msgs::msg::EstimatorStatus::SharedPtr msg)
// {
//     estimator_status_received_ = true;

//     const bool position_valid =
//         msg->pos_horiz_rel_status_flag || msg->pos_horiz_abs_status_flag;
//     const bool attitude_valid = msg->attitude_status_flag;
//     const bool previous_ekf_valid = ekf_valid_;

//     ekf_valid_ = position_valid && attitude_valid;

//     if (ekf_valid_ != previous_ekf_valid)
//     {
//         RCLCPP_INFO(
//             get_logger(),
//             "EKF horizontal position and attitude are now %s",
//             ekf_valid_ ? "valid" : "invalid");
//     }
// }

void RoverMavlinkNode::rf2oCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!isRf2oMessageFinite(*msg))
    {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "RF2O message contains NaN or infinity; message ignored");
        return;
    }

    rf2o_received_ = true;
    last_rf2o_time_ = now();

    if (mavros_odometry_publisher_)
    {
        const auto current_time = now();
        const double min_interval =
            (target_odom_rate_hz_ > 0.0) ? (1.0 / target_odom_rate_hz_) : 0.0;

        if (min_interval > 0.0 &&
            (current_time - last_odom_publish_time_).seconds() < min_interval)
        {
            return;
        }
        last_odom_publish_time_ = current_time;

        nav_msgs::msg::Odometry forwarded = *msg;

        // Zero timestamp forces ArduPilot FCU to stamp with onboard time
        forwarded.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
        forwarded.header.frame_id = odom_frame_id_;
        forwarded.child_frame_id = child_frame_id_;

        // Fill non-zero diagonal covariance 6x6 required by ArduPilot EKF3
        std::array<double, 36> pose_cov = {0.0};
        pose_cov[0]  = 0.01;  // X var [m^2]
        pose_cov[7]  = 0.01;  // Y var [m^2]
        pose_cov[14] = 0.10;  // Z var [m^2]
        pose_cov[21] = 0.05;  // Roll var [rad^2]
        pose_cov[28] = 0.05;  // Pitch var [rad^2]
        pose_cov[35] = 0.02;  // Yaw var [rad^2]
        forwarded.pose.covariance = pose_cov;

        std::array<double, 36> twist_cov = {0.0};
        twist_cov[0]  = 0.02; // Vx
        twist_cov[7]  = 0.02; // Vy
        twist_cov[14] = 0.10; // Vz
        twist_cov[21] = 0.05; // dRoll
        twist_cov[28] = 0.05; // dPitch
        twist_cov[35] = 0.05; // dYaw
        forwarded.twist.covariance = twist_cov;

        mavros_odometry_publisher_->publish(forwarded);
    }
}

void RoverMavlinkNode::heartbeatCallback()
{
    if (autonomous_state_ != AutonomousState::ACTIVE)
    {
        return;
    }

    if (!command_received_)
    {
        if (!heartbeat_triggered_)
        {
            RCLCPP_ERROR(get_logger(), "No navigation command received; holding rover stopped");
            ardupilot_->stop();
            heartbeat_triggered_ = true;
        }
        return;
    }

    const rclcpp::Duration age = now() - last_command_time_;
    if (age.seconds() > heartbeat_timeout_ && !heartbeat_triggered_)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Navigation command timeout after %.2f s; stopping rover",
            age.seconds());
        ardupilot_->stop();
        heartbeat_triggered_ = true;
    }
}

void RoverMavlinkNode::autonomousStateCallback()
{
    if (!ardupilot_->isConnected())
    {
        if (autonomous_state_ != AutonomousState::IDLE)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for MAVROS/FCU connection");
        }
        autonomous_state_ = AutonomousState::IDLE;
        return;
    }

    if (!isAutonomousRequested() &&
        autonomous_state_ != AutonomousState::IDLE &&
        autonomous_state_ != AutonomousState::REQUEST_MANUAL)
    {
        ardupilot_->stop();
        autonomous_state_ = AutonomousState::REQUEST_MANUAL;
    }

    switch (autonomous_state_)
    {
        case AutonomousState::IDLE:
            if (isAutonomousRequested())
            {
                RCLCPP_INFO(get_logger(), "Starting autonomous state machine");
                autonomous_state_ = AutonomousState::REQUEST_GUIDED;
            }
            break;

        case AutonomousState::REQUEST_GUIDED:
            RCLCPP_INFO(get_logger(), "Requesting GUIDED mode");
            ardupilot_->setMode("GUIDED");
            state_deadline_ = now() + rclcpp::Duration::from_seconds(guided_timeout_);
            autonomous_state_ = AutonomousState::WAIT_FOR_GUIDED;
            break;

        case AutonomousState::WAIT_FOR_GUIDED:
            if (ardupilot_->isGuided())
            {
                autonomous_state_ = AutonomousState::CHECK_EKF;
            }
            else if (now() >= state_deadline_)
            {
                RCLCPP_WARN(get_logger(), "GUIDED was not accepted; checking EKF state");
                autonomous_state_ = AutonomousState::CHECK_EKF;
            }
            break;


        case AutonomousState::CHECK_EKF:
            {
                if (!isAutonomousRequested())
                {
                    autonomous_state_ = AutonomousState::REQUEST_MANUAL;
                    break;
                }

                if (isPositionEstimateValid())
                {
                    if (!ardupilot_->isGuided())
                    {
                        failAutonomous(
                            "Local position is valid, but GUIDED was rejected");
                    }
                    else
                    {
                        RCLCPP_INFO(
                            get_logger(),
                            "Local position is valid");

                        autonomous_state_ =
                            AutonomousState::CHECK_ARMED;
                    }

                    break;
                }

                // Local position is unavailable, try RF2O fallback.
                if (isRf2oHealthy())
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Local position is unavailable; enabling RF2O ExternalNav");

                    autonomous_state_ =
                        AutonomousState::ENABLE_EXTERNAL_NAV;
                }
                else
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Local position is unavailable; waiting for RF2O");

                    state_deadline_ =
                        now() +
                        rclcpp::Duration::from_seconds(ekf_timeout_);

                    autonomous_state_ =
                        AutonomousState::WAIT_FOR_RF2O;
                }

                break;
            }
        case AutonomousState::WAIT_FOR_RF2O:
            if (isRf2oHealthy())
            {
                autonomous_state_ = AutonomousState::ENABLE_EXTERNAL_NAV;
            }
            else if (now() >= state_deadline_)
            {
                failAutonomous("RF2O fallback is unavailable or stale");
            }
            break;

        case AutonomousState::ENABLE_EXTERNAL_NAV:
            RCLCPP_WARN(get_logger(), "Selecting EKF SRC2 for RF2O ExternalNav");
            if (!ardupilot_->setEkfSourceSet(2U))
            {
                failAutonomous("Could not send EKF source selection command");
                break;
            }
            external_nav_selected_ = true;
            state_deadline_ = now() + rclcpp::Duration::from_seconds(ekf_timeout_);
            autonomous_state_ = AutonomousState::WAIT_FOR_EKF;
            break;

        case AutonomousState::WAIT_FOR_EKF:
            if (!isRf2oHealthy())
            {
                failAutonomous("RF2O was lost while waiting for EKF");
            }
            else if (isPositionEstimateValid())
            {
                RCLCPP_INFO(get_logger(), "EKF is valid using RF2O ExternalNav");
                autonomous_state_ = AutonomousState::REQUEST_GUIDED;
            }
            else if (now() >= state_deadline_)
            {
                failAutonomous("EKF remained invalid after selecting SRC2");
            }
            break;

        case AutonomousState::CHECK_ARMED:
            if (ardupilot_->isArmed())
            {
                arm_attempts_ = 0;
                heartbeat_triggered_ = false;
                RCLCPP_INFO(get_logger(), "Autonomous control ACTIVE");
                autonomous_state_ = AutonomousState::ACTIVE;
            }
            else if (!auto_arm_)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Waiting for manual arming; set auto_arm:=true to permit automatic arming");
            }
            else
            {
                autonomous_state_ = AutonomousState::REQUEST_ARM;
            }
            break;

        case AutonomousState::REQUEST_ARM:
            if (arm_attempts_ >= max_arm_attempts_)
            {
                failAutonomous("Maximum number of arming attempts reached");
                break;
            }
            ++arm_attempts_;
            RCLCPP_INFO(
                get_logger(),
                "Arming attempt %d/%d",
                arm_attempts_,
                max_arm_attempts_);
            ardupilot_->arm(true);
            state_deadline_ = now() + rclcpp::Duration::from_seconds(arm_timeout_);
            autonomous_state_ = AutonomousState::WAIT_FOR_ARM;
            break;

        case AutonomousState::WAIT_FOR_ARM:
            if (ardupilot_->isArmed())
            {
                autonomous_state_ = AutonomousState::CHECK_ARMED;
            }
            else if (now() >= state_deadline_)
            {
                autonomous_state_ = AutonomousState::REQUEST_ARM;
            }
            break;

        case AutonomousState::ACTIVE:
            {
                if (!isPositionEstimateValid())
                {
                    failAutonomous(
                        "MAVROS local position watchdog expired");
                }
                else if (external_nav_selected_ && !isRf2oHealthy())
                {
                    failAutonomous(
                        "RF2O watchdog expired during autonomous operation");
                }
                else if (!ardupilot_->isGuided())
                {
                    ardupilot_->stop();

                    RCLCPP_WARN(
                        get_logger(),
                        "GUIDED mode was lost; requesting it again");

                    autonomous_state_ =
                        AutonomousState::REQUEST_GUIDED;
                }

                break;
            }

        case AutonomousState::REQUEST_MANUAL:
            ardupilot_->stop();
            ardupilot_->setMode("MANUAL");
            if (external_nav_selected_)
            {
                RCLCPP_INFO(get_logger(), "Restoring EKF source set 1");
                ardupilot_->setEkfSourceSet(1U);
                external_nav_selected_ = false;
            }
            arm_attempts_ = 0;
            autonomous_state_ = AutonomousState::IDLE;
            break;

        case AutonomousState::FAILED:
            if (!isAutonomousRequested())
            {
                autonomous_state_ = AutonomousState::REQUEST_MANUAL;
            }
            break;
    }
}

bool RoverMavlinkNode::isAutonomousRequested() const
{
    return autonomous_enabled_;
}

bool RoverMavlinkNode::isRf2oHealthy() const
{
    if (!rf2o_received_)
    {
        return false;
    }

    return (now() - last_rf2o_time_).seconds() < rf2o_timeout_;
}

bool RoverMavlinkNode::isRf2oMessageFinite(
    const nav_msgs::msg::Odometry & msg) const
{
    return
        std::isfinite(msg.pose.pose.position.x) &&
        std::isfinite(msg.pose.pose.position.y) &&
        std::isfinite(msg.pose.pose.position.z) &&
        std::isfinite(msg.pose.pose.orientation.x) &&
        std::isfinite(msg.pose.pose.orientation.y) &&
        std::isfinite(msg.pose.pose.orientation.z) &&
        std::isfinite(msg.pose.pose.orientation.w) &&
        std::isfinite(msg.twist.twist.linear.x) &&
        std::isfinite(msg.twist.twist.linear.y) &&
        std::isfinite(msg.twist.twist.angular.z);
}

void RoverMavlinkNode::failAutonomous(const std::string & reason)
{
    RCLCPP_ERROR(get_logger(), "Autonomous state machine failed: %s", reason.c_str());
    ardupilot_->stop();
    autonomous_state_ = AutonomousState::FAILED;
}


void RoverMavlinkNode::localPositionCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    const auto & p = msg->pose.position;
    const auto & q = msg->pose.orientation;

    const bool finite =
        std::isfinite(p.x) &&
        std::isfinite(p.y) &&
        std::isfinite(p.z) &&
        std::isfinite(q.x) &&
        std::isfinite(q.y) &&
        std::isfinite(q.z) &&
        std::isfinite(q.w);

    if (!finite)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Invalid local position received");
        return;
    }

    local_position_received_ = true;
    last_local_position_time_ = this->now();
}

bool RoverMavlinkNode::isPositionEstimateValid() const
{
    if (!local_position_received_)
    {
        return false;
    }

    return
        (this->now() - last_local_position_time_).seconds()
        <= local_position_timeout_;
}