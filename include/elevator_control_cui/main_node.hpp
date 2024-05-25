#ifndef MAIN_NODE_HPP
#define MAIN_NODE_HPP


// Copyright 2023 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MAIN_NODE_HPP_
#define MAIN_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>

#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <thread>
#include <ctime>
#include <cstdlib>
#include <utility>

#include <codecvt>
#include <ncursesw/curses.h>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "elevator_interfaces/msg/result_check_elevator_status.hpp"
#include "elevator_interfaces/msg/robot_service_sequence.hpp"
#include "elevator_interfaces/srv/call_elevator_service.hpp"
#include "elevator_interfaces/srv/call_robot_service.hpp"
#include "elevator_interfaces/srv/call_robot_service_in_ev.hpp"
#include "elevator_interfaces/srv/cancel_robot_service.hpp"
#include "elevator_interfaces/srv/check_elevator_status.hpp"
#include "elevator_interfaces/srv/get_elevator_status.hpp"
#include "elevator_interfaces/srv/get_robot_service_status.hpp"
#include "elevator_interfaces/srv/hold_door.hpp"
#include "elevator_interfaces/srv/set_robot_service.hpp"
#include "elevator_interfaces/srv/add_robot_service_timeout.hpp"
#include "elevator_interfaces/srv/get_robot_service_last_sequence.hpp"
#include "elevator_interfaces/srv/set_elevator_group_control.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

class MainNode : public rclcpp::Node
{
public:
  MainNode();
  ~MainNode();

  void init_window();
  void init_color_pair();
  void input_handler(int c);
  void init_window_format();
  void init_ev_status_window();
  void init_robot_service_window();
  void clear_robot_service_item(int index);
  void set_robot_service_item_on(int index);
  void init_set_sequence_window();
  void init_get_status_window();
  void set_get_status_item_on(int index);
  void clear_get_status_item(int index);
  void update_sequence();
  void set_send_sequence_item_on(int index);
  void init_menu_window();
  void set_menu_item_on(int index);
  void get_keyboard_input(int & c);

  void print_test(std::string value);
  void print_ev_num(std::string value);
  void print_ev_name(std::string value);
  void print_floor(std::string value);
  void print_direction(std::string value);
  void print_run(std::string value);
  void print_door(std::string value);
  void print_mode(std::string value);
  void print_in_ev(bool value);
  void print_ev_status();

  void print_call_ev_num(std::string value);
  void print_call_floor(std::string value);
  void print_dest_floor(std::string value);
  void print_sequence(std::string value);
  void print_service_result(std::string value);

  void update_topic_recv_time();

  void robot_service_seqeunce_callback(
    const elevator_interfaces::msg::RobotServiceSequence::SharedPtr msg);

  // Robot Service Callers
  void robot_service_call(int ev_num, std::string call_floor, std::string dest_floor);
  void robot_service_in_ev_call(int ev_num, std::string dest_floor);
  void cancel_robot_service();
  void get_ev_status(int ev_num);
  void set_robot_service(std::string robot_status);

  std::string array_msg_to_string(std::vector<int8_t> array_msg);
  std::string array_msg_to_string(std::vector<int32_t> array_msg);
  std::string array_msg_to_string(std::vector<std::string> array_msg);

  // ROS Subscriber
  rclcpp::Subscription<elevator_interfaces::msg::RobotServiceSequence>::SharedPtr
    sequence_subscriber;

  // ROS Service Client
  rclcpp::Client<elevator_interfaces::srv::CallRobotService>::SharedPtr call_robot_service_client;
  rclcpp::Client<elevator_interfaces::srv::CallRobotServiceInEV>::SharedPtr
    call_robot_service_in_ev_client;
  rclcpp::Client<elevator_interfaces::srv::CallElevatorService>::SharedPtr call_ev_service_client;
  rclcpp::Client<elevator_interfaces::srv::CancelRobotService>::SharedPtr
    cancel_robot_service_client;
  rclcpp::Client<elevator_interfaces::srv::GetElevatorStatus>::SharedPtr get_ev_status_client;
  rclcpp::Client<elevator_interfaces::srv::SetRobotService>::SharedPtr set_robot_status_client;

private:
  const int UPPER_WIN_START_X = 1;
  const int UPPER_WIN_START_Y = 0;
  const int LOWER_WIN_START_X = 1;
  const int LOWER_WIN_START_Y = 9;

  bool in_ev_ = false;
  std::string call_ev_num_;
  std::string call_floor_ = "";
  std::string dest_floor_ = "";
  std::string sequence_ = "";

  std::string ev_num_ = "";
  std::string ev_name_ = "";
  std::string floor_ = "";
  std::string direction_ = "";
  std::string run_ = "";
  std::string door_ = "";
  std::string mode_ = "";

  std::string robot_service_result_ = "";

  std::string topic_recv_time_ = "";

  int get_status_ev_num_ = 0;
  bool repeat_get_status_ = false;
  int get_status_interval_sec_ = 1;

  WINDOW * sub_window = nullptr;
  WINDOW * ev_status_window_ = nullptr;
  WINDOW * ev_status_sub_window_ = nullptr;
  WINDOW * robot_service_window_ = nullptr;
  WINDOW * set_sequence_window_ = nullptr;
  WINDOW * get_status_window_ = nullptr;
  WINDOW * menu_window_ = nullptr;

  const int COLOR_PAIR_BG = 0;
  const int COLOR_PAIR_WHITE_BLUE_ = 1;
  const int COLOR_PAIR_BLUE_GREEN_ = 2;
  const int COLOR_PAIR_BLACK_GREEN_ = 3;
  const int COLOR_PAIR_MENU_ = 4;
  const int COLOR_PAIR_MENU_ITEM_ = 5;
  const int COLOR_PAIR_MENU_ITEM_ON_ = 6;
  const int COLOR_PAIR_ROBOT_SERVICE_DATA_ = 7;
  const int COLOR_PAIR_ROBOT_SERVICE_ITEM_ON_ = 8;


  enum MenuItem {ROBOT_SERVICE = 0, SET_SEQUENCE, GET_EV_STATUS};
  enum RobotServiceItem {EV_NUM = 0, CALL_FLOOR, DEST_FLOOR, IV_EV};
  enum SetSequenceItem {TAKING_ON = 0, GETTING_OFF};
  enum GetStatusItem {GET_STATUS_EV_NUM=0, REPEAT, INTERVAL, GET_STATUS_ONCE};

  int cnt_ = 0;
  int curr_menu_index_ = 0;
  int curr_robot_service_item_ = 0;
  int curr_sequnce_item_ = 0;
  int curr_get_status_item_ = 0;
};

#endif  // TKE_CONTROLLER__TKE_CONTROLLER_NODE_HPP_


#endif // MAIN_NODE_HPP
