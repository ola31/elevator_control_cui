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

};

#endif  // TKE_CONTROLLER__TKE_CONTROLLER_NODE_HPP_


#endif // MAIN_NODE_HPP
