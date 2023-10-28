#include "iris_gazebo/robot_transmission.hpp"

using namespace gazebo;
using namespace ignition;

RobotGazebo::RobotGazebo()
{
    ROS_ERROR("Robot Velocity Plugin Loaded!");
}

RobotGazebo::~RobotGazebo()
{
    rosnode->shutdown();
}

void RobotGazebo::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    model = _model;
    model_name = model->GetName();
    ROS_ERROR("Hello World! %s", model_name.c_str());

    SetMotorToCenterDistance();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RobotGazebo::OnUpdate, this, _1));

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "robot_client", ros::init_options::NoSigintHandler);
    }
    rosnode = new ros::NodeHandle();

    // Subscribe to cmd_vel topic
    cmd_vel_sub = rosnode->subscribe("cmd_vel", 1, &RobotGazebo::cmdVelCallback, this);
}

void RobotGazebo::OnUpdate(const common::UpdateInfo & /*_info*/)
{
    // Step 1: Buffer the desired velocities
    double left_velocity = robot.vel_left;
    double right_velocity = robot.vel_right;
    double rear_velocity = robot.vel_rear;
    double slider_position = (robot.vel_left == 0 && robot.vel_right == 0 && robot.vel_rear == 0) ? -0.05 : 0;

    // Step 2: Apply the buffered velocities in rapid succession
    model->GetJoint("left_motor_revolute")->SetVelocity(0, left_velocity);   // Set velocity units=rad/s
    model->GetJoint("right_motor_revolute")->SetVelocity(0, right_velocity); // Set velocity units=rad/s
    model->GetJoint("rear_motor_revolute")->SetVelocity(0, rear_velocity);   // Set velocity units=rad/s

    // Apply the buffered position
    model->GetJoint("Slider 56")->SetPosition(0, slider_position);

    math::Vector3d robot_pose = model->RelativePose().Pos();
}

void RobotGazebo::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    ROS_INFO("Received a /cmd_vel message! %f %f %f", msg->linear.x, msg->linear.y, msg->angular.z);
    // Extract linear x velocity for demonstration purposes
    double velocity_x = msg->linear.x;
    double velocity_y = msg->linear.y;
    double velocity_th = msg->angular.z;
    SetVelocityRobot(velocity_x, velocity_y, velocity_th);
}

void RobotGazebo::SetVelocityRobot(float vx, float vy, float vth)
{
    // this equation is wrong
    double omega = vth;
    double updated_th = vth * DEG2RAD * 24 * AUX_CONST;
    // ROS_ERROR("%f %f", vx * cosf(LEFT_MOTOR_ANGLE * DEG2RAD) + vy * sinf(LEFT_MOTOR_ANGLE * DEG2RAD), vx * cosf(RIGHT_MOTOR_ANGLE * DEG2RAD) + vy * sinf(RIGHT_MOTOR_ANGLE * DEG2RAD));
    // robot.vel_left = updated_th + vx * cosf(LEFT_MOTOR_ANGLE * DEG2RAD) + vy * sinf(LEFT_MOTOR_ANGLE * DEG2RAD);
    // robot.vel_right = updated_th + vx * cosf(LEFT_MOTOR_ANGLE * DEG2RAD) + vy * sinf(LEFT_MOTOR_ANGLE * DEG2RAD);
    // robot.vel_rear = updated_th + vy;

    robot.vel_left = -0.5 * vx + sqrt(3) / 2 * vy + left_motor_to_center * omega; // Assuming AUX_CONST is L0 from the paper
    robot.vel_right = (-0.5 * vx + (sqrt(3) / 2 * vy) + right_motor_to_center * omega) * (vx != 0 ? -1 : 1);
    robot.vel_rear = vx + rear_motor_to_center * omega;

    // robot.vel_left = sqrt(3) / 2 * vy + left_motor_to_center * omega;
    // robot.vel_right = sqrt(3) / 2 * vy + right_motor_to_center * omega;
    // robot.vel_rear = vx + rear_motor_to_center * omega;

    ROS_WARN("UPDATED VELOCITY: %f %f %f", robot.vel_left, robot.vel_right, robot.vel_rear);
}

void RobotGazebo::SetMotorToCenterDistance()
{
    math::Vector3d left_motor_pos = model->GetJoint("left_motor_revolute")->GetChild()->RelativePose().Pos();
    math::Vector3d right_motor_pos = model->GetJoint("right_motor_revolute")->GetChild()->RelativePose().Pos();
    math::Vector3d rear_motor_pos = model->GetJoint("rear_motor_revolute")->GetChild()->RelativePose().Pos();
    math::Vector3d center_pos = model->RelativePose().Pos();
    std::cout << "left_motor_pos: " << left_motor_pos << " right_motor_pos: " << right_motor_pos << " rear_motor_pos: " << rear_motor_pos << " center_pos: " << center_pos << std::endl;

    left_motor_to_center = sqrt(pow(left_motor_pos.X() - center_pos.X(), 2) + pow(left_motor_pos.Y() - center_pos.Y(), 2));
    right_motor_to_center = sqrt(pow(right_motor_pos.X() - center_pos.X(), 2) + pow(right_motor_pos.Y() - center_pos.Y(), 2));
    rear_motor_to_center = sqrt(pow(rear_motor_pos.X() - center_pos.X(), 2) + pow(rear_motor_pos.Y() - center_pos.Y(), 2));
    std::cout << "left_motor_to_center: " << left_motor_to_center << " right_motor_to_center: " << right_motor_to_center << " rear_motor_to_center: " << rear_motor_to_center << std::endl;
}