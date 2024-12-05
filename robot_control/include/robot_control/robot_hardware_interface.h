#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

class ROBOTHardwareInterface : public hardware_interface::RobotHW {
public:
    ROBOTHardwareInterface(ros::NodeHandle& nh)
        : nh_(nh), loop_hz_(10.0) {
        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        non_realtime_loop_ = nh_.createTimer(ros::Duration(1.0 / loop_hz_), &ROBOTHardwareInterface::update, this);
    }

    ~ROBOTHardwareInterface() {}

    void init() {
        // Register joint state interface
        hardware_interface::JointStateHandle state_handle_left(joint_name_[0], &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
        hardware_interface::JointStateHandle state_handle_right(joint_name_[1], &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
        joint_state_interface_.registerHandle(state_handle_left);
        joint_state_interface_.registerHandle(state_handle_right);
        registerInterface(&joint_state_interface_);

        // Register velocity joint interface
        hardware_interface::JointHandle vel_handle_left(joint_state_interface_.getHandle(joint_name_[0]), &joint_velocity_command_[0]);
        hardware_interface::JointHandle vel_handle_right(joint_state_interface_.getHandle(joint_name_[1]), &joint_velocity_command_[1]);
        velocity_joint_interface_.registerHandle(vel_handle_left);
        velocity_joint_interface_.registerHandle(vel_handle_right);
        registerInterface(&velocity_joint_interface_);

        // ROS Publishers and Subscribers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        left_encoder_sub_ = nh_.subscribe("left_ticks", 10, &ROBOTHardwareInterface::leftEncoderCallback, this);
        right_encoder_sub_ = nh_.subscribe("right_ticks", 10, &ROBOTHardwareInterface::rightEncoderCallback, this);

        // Initialize joint values
        joint_position_[0] = joint_position_[1] = 0.0;
        joint_velocity_[0] = joint_velocity_[1] = 0.0;
        joint_effort_[0] = joint_effort_[1] = 0.0;
        joint_velocity_command_[0] = joint_velocity_command_[1] = 0.0;
    }

    void update(const ros::TimerEvent& e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void read() {
        // Update joint position and velocity from encoder data
        // Assumes encoder ticks are converted to velocity externally
        joint_velocity_[0] = left_motor_velocity_;
        joint_velocity_[1] = right_motor_velocity_;
    }

    void write(ros::Duration elapsed_time) {
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = (joint_velocity_command_[0] + joint_velocity_command_[1]) / 2.0;
        cmd_vel_msg.angular.z = (joint_velocity_command_[1] - joint_velocity_command_[0]) / 0.19; // Adjust wheel_separation
        cmd_vel_pub_.publish(cmd_vel_msg);
    }

    void leftEncoderCallback(const std_msgs::Int16& msg) {
        left_motor_velocity_ = msg.data; // Convert encoder ticks to velocity as needed
    }

    void rightEncoderCallback(const std_msgs::Int16& msg) {
        right_motor_velocity_ = msg.data; // Convert encoder ticks to velocity as needed
    }

protected:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber left_encoder_sub_;
    ros::Subscriber right_encoder_sub_;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    std::string joint_name_[2] = {"left_wheel_joint", "right_wheel_joint"};
    double joint_position_[2];
    double joint_velocity_[2];
    double joint_effort_[2];
    double joint_velocity_command_[2];

    ros::Timer non_realtime_loop_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    ros::Duration elapsed_time_;
    double loop_hz_;

    double left_motor_velocity_ = 0.0;
    double right_motor_velocity_ = 0.0;
};

