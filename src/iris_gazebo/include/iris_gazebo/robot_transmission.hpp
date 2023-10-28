#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

#define CNTS2DEG (360.0f / 13186.0f)
#define DEG2CNTS (13186.0f / 360.0f)
#define AUX_CONST 1.25
#define RAD2DEG (180.0f / M_PI)
#define DEG2RAD (M_PI / 180.0f)
#define LEFT_MOTOR_ANGLE 240
#define RIGHT_MOTOR_ANGLE 120
#define REAR_MOTOR_ANGLE 0

typedef struct Robot
{
    float pose_x;
    float pose_y;
    float pose_th;
    float vel_x;
    float vel_y;
    float vel_th;
    float vel_left;
    float vel_rear;
    float vel_right;
} Robot_Tag;

// using namespace ignition;
namespace gazebo
{
    class RobotGazebo : public ModelPlugin
    {
    private:
        ros::NodeHandle *rosnode; // Pointer to ROS node
        ros::Subscriber cmd_vel_sub;
        Robot_Tag robot;
        physics::ModelPtr model;
        std::string model_name;
        event::ConnectionPtr updateConnection;

        double left_motor_to_center;
        double right_motor_to_center;
        double rear_motor_to_center;

    public:
        RobotGazebo();
        ~RobotGazebo();

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void OnUpdate(const common::UpdateInfo & /*_info*/);
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
        void SetVelocityRobot(float vx, float vy, float vth);
        void SetMotorToCenterDistance();
    };

    GZ_REGISTER_MODEL_PLUGIN(RobotGazebo)
}
