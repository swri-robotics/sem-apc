#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <string>
#include <thread>

#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "carla_msgs/CarlaEgoVehicleStatus.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

using CarCmd = carla_msgs::CarlaEgoVehicleControl;
using Float64 = std_msgs::Float64;

namespace carla_shell_bridge
{
class CarlaSimulationVehicleInterface : public nodelet::Nodelet
{
 public:
  ~CarlaSimulationVehicleInterface() override
  {

  }

  void onInit() override
  {
    ros::NodeHandle pnh = getPrivateNodeHandle();

    // create a one-shot timer to initialize everything after a brief
    // pause so that ros has time to connect to /rosout and we don't
    // drop errors/info during initialization.
    double initialization_delay = 1.0;
    pnh.param("initialization_delay_s", initialization_delay, 1.0);
    init_timer_ = pnh.createWallTimer(ros::WallDuration(initialization_delay),
                                      &CarlaSimulationVehicleInterface::initialize,
                                      this,
                                      true);
  }

 private:

  bool handbrake_ = false;
  bool invert_steering_ = false;
  bool manual_control_ = false;
  float brake_ = 0.0;
  float steering_ = 0.0;
  float throttle_ = 0.0;
  float speed_;

  std::string gear_input_;
  std_msgs::Bool manual_override_msg;
  CarCmd carla_control_;

  // Publishers
  ros::Publisher car_cmd_pub_;
  
  // Subscribers
  ros::Subscriber brake_sub_;
  ros::Subscriber gear_sub_;
  ros::Subscriber steering_sub_;
  ros::Subscriber throttle_sub_;
  ros::Subscriber speedometer_sub_;

  ros::WallTimer update_car_cmd_;
  ros::WallTimer init_timer_;

  void initialize(const ros::WallTimerEvent &)
  {
    // main nodelet code goes here
    auto nh = getMTNodeHandle();
    auto pnh = getMTPrivateNodeHandle();

    if (pnh.hasParam("invert_steering"))
    {
      invert_steering_ = pnh.getParam("invert_steering", invert_steering_);
      ROS_INFO_STREAM("invert steering set to: " << (invert_steering_ ? "true" : "false"));
    }
    else
    {
      ROS_INFO_STREAM("invert steering defaults to: false");
    }

    car_cmd_pub_ = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("carla/ego_vehicle/vehicle_control_cmd", 10);    

    brake_sub_ = nh.subscribe("/brake_command", 1,
                              &CarlaSimulationVehicleInterface::HandleBrakeInput, this);
    gear_sub_ = nh.subscribe("/gear_command", 1,
                              &CarlaSimulationVehicleInterface::HandleTransmissionInput, this);
    steering_sub_ = nh.subscribe("/steering_command", 1,
                                  &CarlaSimulationVehicleInterface::HandleSteeringInput, this);
    throttle_sub_ = nh.subscribe("/throttle_command", 1,
                                  &CarlaSimulationVehicleInterface::HandleThrottleInput, this);

    // should this be a wall timer?
    update_car_cmd_ = nh.createWallTimer(ros::WallDuration(ros::Rate(10.0)),
                                                            &CarlaSimulationVehicleInterface::update,
                                                            this);
    
    ROS_INFO_STREAM("Running CARLA vehicle interface");
  }

  void update(const ros::WallTimerEvent &event)
  {
    auto car_cmd_msg = CarCmd();
    int carla_gear = carla_control_.gear;

    // Set manual override in Carla to true
    manual_override_msg.data = true;
    float covariance_amount = 0.001;

    // limit steering to [-1.0, 1.0]
    car_cmd_msg.steer = (invert_steering_ ? -1.0 : 1.0) * fmin(1.0, fmax(-1.0, steering_));

    // configure throttle, brake, handbrake
    // car_cmd_msg.hand_brake = handbrake_;
    if (handbrake_ || fabs(brake_) >= 0.01)
    {
      // hand brake is engaged, or brake is engaged
      car_cmd_msg.throttle = 0.0;
      car_cmd_msg.brake = brake_;
    }
    else
    {
      car_cmd_msg.throttle = throttle_;
      car_cmd_msg.brake = 0.0;
    }

    // configure gearbox
    if (gear_input_== "reverse")
    {
      // set gearbox to reverse
      car_cmd_msg.gear = -1;
      car_cmd_msg.reverse = true;    
    }
    else if (gear_input_== "forward")
    {
      // default case is forwards
      car_cmd_msg.gear = 1;
      car_cmd_msg.reverse = false;
    } 
    else
    {
      car_cmd_msg.gear = 0;
      car_cmd_msg.reverse = false;
    }
    car_cmd_msg.manual_gear_shift = true;

    if (!manual_control_)
    {
      car_cmd_pub_.publish(car_cmd_msg);
    }
  }

  void HandleBrakeInput(const Float64::ConstPtr &msg)
  {
    brake_ = msg->data;
  }

  void HandleTransmissionInput(const std_msgs::StringConstPtr &msg)
  {
    gear_input_= msg->data;
  }

  void HandleSteeringInput(const Float64::ConstPtr &msg)
  {
    steering_ = msg->data;
  }

  void HandleThrottleInput(const Float64::ConstPtr &msg)
  {
    throttle_ = msg->data;
  }

};  // class CarlaSimulationVehicleInterface
}  //namespace carla_shell_bridge

// Register nodelet plugin
#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(
  carla_shell_bridge,
  CarlaSimulationVehicleInterface)