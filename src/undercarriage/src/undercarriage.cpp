#include <sensor_msgs/msg/joy.hpp>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "undercarriage/undercarriage.hpp"
#include "robomas_plugins/msg/robomas_frame.hpp"
#include "robomas_plugins/msg/robomas_target.hpp"
#include "robomas_plugins/msg/frame.hpp"
// #include "controllernode/msg/autotarget.hpp"

using std::placeholders::_1;
enum class robot_mode
{
  disable, controller, automode
};

class Undercarriage_Node: public rclcpp::Node
{
public:
  Undercarriage_Node()
  : Node("undercarriage")
  {
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Undercarriage_Node::joy_callback, this, _1));
      // auto_move_subscription_ = this->create_subscription<controllernode::msg::Autotarget>(
      // "/auto_target", 10, std::bind(&Undercarriage_Node::auto_move_callback, this));
      setting_frame_publisher_ = this->create_publisher<robomas_plugins::msg::RobomasFrame>("robomas_frame", 10);
      motors.push_back({
        Motor(1.0, 0.0, 0,this->now()),
        this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target0", 10)
      });
      motors.push_back({
        Motor(-0.5, -Constants::Math::cos30, 1,this->now()),
        this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target1", 10)
      });
      motors.push_back({
        Motor(-0.5, Constants::Math::cos30, 2,this->now()),
        this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target2", 10)
      });
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy &msg)
  {
    if(msg.buttons[7]){//startボタン
      this_robot_mode = robot_mode::controller;
      for(auto& one_motor : this->motors){
        setting_frame_publisher_->publish(std::move(this->make_setting_frame(one_motor.first.get_motor_id(),true)));
      }
    }//mode velにする
    if(msg.buttons[6]){//backボタン 
      this_robot_mode = robot_mode::disable;
      for(auto& one_motor : this->motors){
        setting_frame_publisher_->publish(std::move(this->make_setting_frame(one_motor.first.get_motor_id(),false)));
      }
    }//mode disにする
    if(msg.buttons[0]){//Aボタン
      this_robot_mode = robot_mode::automode;
      for(auto& one_motor : this->motors){
        setting_frame_publisher_->publish(std::move(this->make_setting_frame(one_motor.first.get_motor_id(),true)));
      }
    }
    if(this_robot_mode == robot_mode::controller){
      target_vector.x = msg.axes[0];
      target_vector.y = msg.axes[1];
      float rotation = 0.0f;
      if(msg.axes[2] == -1){
        rotation += Constants::Undercarriage::ROBOT_ROTATION_VELOCITY;
      }
      if(msg.axes[5] == -1){
        rotation -= Constants::Undercarriage::ROBOT_ROTATION_VELOCITY;
      }
      for(auto& one_motor : motors){
        robomas_plugins::msg::RobomasTarget target_msg;
        target_msg.target  = one_motor.first.update(target_vector * one_motor.first.get_vec2d() * Constants::Undercarriage::ROBOT_VELOCITY + rotation, this->now());
        one_motor.second->publish(target_msg);
      }
    }
  }

  // void auto_move_callback(const controllernode::msg::Autotarget &msg){
  //   if(this_robot_mode == robot_mode::automode){
  //     target_vector.x = msg.x;
  //     target_vector.y = msg.y;
  //     for(auto& one_motor : motors){
  //       robomas_plugins::msg::RobomasTarget target_msg;
  //       target_msg.target  = one_motor.first.update(target_vector * one_motor.first.get_vec2d() * Constants::Undercarriage::ROBOT_VELOCITY, this->now());
  //       one_motor.second->publish(target_msg);
  //     }
  //   }
  // }
  robomas_plugins::msg::RobomasFrame  make_setting_frame(uint8_t motor_number,bool MODE ){
    robomas_plugins::msg::RobomasFrame undercarriage_frame;
    undercarriage_frame.motor = motor_number;
    undercarriage_frame.temp = Constants::Undercarriage::temp;
    undercarriage_frame.c620 = Constants::Undercarriage::c620;
    undercarriage_frame.mode = MODE;
    undercarriage_frame.velkp = Constants::Undercarriage::velkp;
    undercarriage_frame.velki = Constants::Undercarriage::velki;
    undercarriage_frame.poskp = Constants::Undercarriage::poskp;
    undercarriage_frame.tyoku_vel_target = Constants::Undercarriage::tyoku_vel_target;
    undercarriage_frame.tyoku_pos_target = Constants::Undercarriage::tyoku_pos_target;
    undercarriage_frame.stable_pos_limit_vel = Constants::Undercarriage::stable_pos_limit_vel;
    return undercarriage_frame;
}

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasFrame>::SharedPtr setting_frame_publisher_;
  // rclcpp::Subscription<controllernode::msg::Autotarget>::SharedPtr auto_move_subscription_;
  std::vector<std::pair<Motor,rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr>> motors;
  // bool robot_mode = false;
  robot_mode this_robot_mode = robot_mode::disable;
  FRY::Vec2d target_vector;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Undercarriage_Node>());
  rclcpp::shutdown();
  return 0;
}
