#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <string>
#include <memory>
#include <vector>
// #include <ncurses.h>
#include <locale.h>
#include <time.h>

#include "elevator_control_cui/main_node.hpp"


MainNode::MainNode()
: rclcpp::Node("elevator_control_cui")
{
  RCLCPP_INFO(this->get_logger(), "elevator_control_cui");

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort().durability_volatile();
  using namespace std::placeholders;

  // Subscriber
  sequence_subscriber = this->create_subscription<elevator_interfaces::msg::RobotServiceSequence>(
    "/robot_service_sequence", qos_profile, std::bind(
      &MainNode::robot_service_seqeunce_callback, this, _1));

  // Service
  call_robot_service_client = this->create_client<elevator_interfaces::srv::CallRobotService>(
    "call_robot_service");
  call_robot_service_in_ev_client =
    this->create_client<elevator_interfaces::srv::CallRobotServiceInEV>(
    "call_robot_service_in_ev");

  call_ev_service_client = this->create_client<elevator_interfaces::srv::CallElevatorService>(
    "call_elevator_service");

  cancel_robot_service_client = this->create_client<elevator_interfaces::srv::CancelRobotService>(
    "cancel_robot_service");

  get_ev_status_client = this->create_client<elevator_interfaces::srv::GetElevatorStatus>(
    "get_elevator_status");

  set_robot_status_client = this->create_client<elevator_interfaces::srv::SetRobotService>(
    "set_robot_service");


  init_window();
}

MainNode::~MainNode()
{
  std::cout << "terminate..." << std::endl;
  delwin(ev_status_sub_window_);
  delwin(ev_status_window_);
  delwin(robot_service_window_);
  delwin(sub_window);
  delwin(menu_window_);
  endwin();
}


void MainNode::update_topic_recv_time()
{
  // 현재 시간을 시간점으로 얻기
  auto now = std::chrono::system_clock::now();

  // 시간점을 초 단위로 변환하기
  auto now_seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);

  // 변환된 시간점을 시간 타입으로 변환하기
  std::time_t now_time_t = std::chrono::system_clock::to_time_t(now_seconds);

  // 시간 타입을 문자열로 변환하기
  topic_recv_time_ = std::to_string(now_time_t);
}

void MainNode::robot_service_seqeunce_callback(
  const elevator_interfaces::msg::RobotServiceSequence::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "sequence : %s", msg->sequence.c_str());
  sequence_ = msg->sequence;
  update_topic_recv_time();
  print_sequence(sequence_);
}


void MainNode::get_ev_status(int ev_num)
{
  auto request = std::make_shared<elevator_interfaces::srv::GetElevatorStatus::Request>();
  request->ev_num = ev_num;

  using ServiceResponseFuture =
    rclcpp::Client<elevator_interfaces::srv::GetElevatorStatus>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      // RCLCPP_INFO(this->get_logger(), "Result : %s", response->result);

      std::vector<int8_t> ev_num = response->ev_num;
      std::vector<std::string> ev_name = response->ev_name;
      std::vector<std::string> floor = response->floor;
      std::vector<std::string> direction = response->direction;
      std::vector<std::string> run = response->run;
      std::vector<std::string> door = response->door;
      std::vector<std::string> mode = response->mode;
      std::vector<int32_t> error_code = response->error_code;
      std::vector<std::string> group = response->group;

      ev_num_ = array_msg_to_string(ev_num);
      ev_name_ = array_msg_to_string(ev_name);
      floor_ = array_msg_to_string(floor);
      direction_ = array_msg_to_string(direction);
      run_ = array_msg_to_string(run);
      door_ = array_msg_to_string(door);
      mode_ = array_msg_to_string(mode);

      print_ev_status();
      return;
    };

  auto future_result =
    get_ev_status_client->async_send_request(request, response_received_callback);
}


void MainNode::robot_service_call(int ev_num, std::string call_floor, std::string dest_floor)
{
  auto request = std::make_shared<elevator_interfaces::srv::CallRobotService::Request>();
  request->ev_num = ev_num;
  request->call_floor = call_floor;
  request->dest_floor = dest_floor;

  using ServiceResponseFuture =
    rclcpp::Client<elevator_interfaces::srv::CallRobotService>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      // RCLCPP_INFO(this->get_logger(), "Result : %s", response->result.c_str());
      bool result = response->result;
      if (result) {
        robot_service_result_ = "True";
      } else {
        robot_service_result_ = "False";
      }
    };

  auto future_result =
    call_robot_service_client->async_send_request(request, response_received_callback);
}


void MainNode::robot_service_in_ev_call(int ev_num, std::string dest_floor)
{
  auto request = std::make_shared<elevator_interfaces::srv::CallRobotServiceInEV::Request>();
  request->ev_num = ev_num;
  request->dest_floor = dest_floor;

  using ServiceResponseFuture =
    rclcpp::Client<elevator_interfaces::srv::CallRobotServiceInEV>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      // RCLCPP_INFO(this->get_logger(), "Result : %s", response->result.c_str());
      bool result = response->result;
      if (result) {
        robot_service_result_ = "True";
      } else {
        robot_service_result_ = "False";
      }
      return;
    };

  auto future_result =
    call_robot_service_in_ev_client->async_send_request(request, response_received_callback);
}

void MainNode::cancel_robot_service()
{
  auto request = std::make_shared<elevator_interfaces::srv::CancelRobotService::Request>();

  using ServiceResponseFuture =
    rclcpp::Client<elevator_interfaces::srv::CancelRobotService>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      return;
    };

  auto future_result =
    cancel_robot_service_client->async_send_request(request, response_received_callback);
}

void MainNode::set_robot_service(std::string robot_status)
{
  auto request = std::make_shared<elevator_interfaces::srv::SetRobotService::Request>();
  request->robot_status = robot_status;

  using ServiceResponseFuture =
    rclcpp::Client<elevator_interfaces::srv::SetRobotService>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      // RCLCPP_INFO(this->get_logger(), "Result : %s", response->result);
      // bool result = response->result;
      std::string ev_service_status = response->ev_service_status;
      return;
    };

  auto future_result =
    set_robot_status_client->async_send_request(request, response_received_callback);
}


std::string MainNode::array_msg_to_string(std::vector<int8_t> array_msg)
{
  std::string result = std::string("[");
  for (size_t i = 0; i < array_msg.size(); i++) {
    result = result + std::to_string(array_msg[i]);
    if (i < array_msg.size() - 1) {
      result = result + ", ";
    }
  }
  result = result + "]";
  return result;
}


std::string MainNode::array_msg_to_string(std::vector<int32_t> array_msg)
{
  std::string result = std::string("[");
  for (size_t i = 0; i < array_msg.size(); i++) {
    result = result + std::to_string(array_msg[i]);
    if (i < array_msg.size() - 1) {
      result = result + ", ";
    }
  }
  result = result + "]";
  return result;
}


std::string MainNode::array_msg_to_string(std::vector<std::string> array_msg)
{
  std::string result = std::string("[");
  for (size_t i = 0; i < array_msg.size(); i++) {
    result = result + array_msg[i];
    if (i < array_msg.size() - 1) {
      result = result + ", ";
    }
  }
  result = result + "]";
  return result;
}


void MainNode::init_window()
{
  initscr();
  // Keyboard Setting
  keypad(stdscr, TRUE);     // 특수문자 입력 가능
  curs_set(0);     // 커서 사라짐
  noecho();     // 입력문자 안보이게
  start_color();
  init_color_pair();

  bkgd(COLOR_PAIR(COLOR_PAIR_BG));
  refresh();

//  print_ev_status();
  init_ev_status_window();
  init_robot_service_window();
  init_menu_window();
  init_window_format();
}

void MainNode::init_window_format()
{
  mvwprintw(ev_status_window_, UPPER_WIN_START_Y + 1, UPPER_WIN_START_X, "ev_num    ");
  mvwprintw(ev_status_window_, UPPER_WIN_START_Y + 2, UPPER_WIN_START_X, "ev_name   ");
  mvwprintw(ev_status_window_, UPPER_WIN_START_Y + 3, UPPER_WIN_START_X, "floor     ");
  mvwprintw(ev_status_window_, UPPER_WIN_START_Y + 4, UPPER_WIN_START_X, "direction ");
  mvwprintw(ev_status_window_, UPPER_WIN_START_Y + 5, UPPER_WIN_START_X, "run       ");
  mvwprintw(ev_status_window_, UPPER_WIN_START_Y + 6, UPPER_WIN_START_X, "door      ");
  mvwprintw(ev_status_window_, UPPER_WIN_START_Y + 7, UPPER_WIN_START_X, "mode      ");

//  mvwprintw(ev_status_window_, UPPER_WIN_START_Y + 9, UPPER_WIN_START_X, "ev_num     : ");
//  mvwprintw(ev_status_window_, UPPER_WIN_START_Y + 10, UPPER_WIN_START_X, "call_floor : ");
//  mvwprintw(ev_status_window_, UPPER_WIN_START_Y + 11, UPPER_WIN_START_X, "dest_floor : ");
//  mvwprintw(ev_status_window_, UPPER_WIN_START_Y + 12, UPPER_WIN_START_X, "In EV      : [ ]");
//  mvwprintw(ev_status_window_, UPPER_WIN_START_Y + 13, UPPER_WIN_START_X, "Sequence   : ");
//  mvwprintw(ev_status_window_, UPPER_WIN_START_Y + 14, UPPER_WIN_START_X, "result     : ");
  //mvprintw(UPPER_WIN_START_Y + 14, UPPER_WIN_START_X, "");
//  /mvprintw(UPPER_WIN_START_Y + 15, UPPER_WIN_START_X, "");
  mvprintw(
    UPPER_WIN_START_Y + 16, UPPER_WIN_START_X,
    "Enter : get ev status");

  mvprintw(
    UPPER_WIN_START_Y + 17, UPPER_WIN_START_X,
    "T : Taking On  G : Getting Off");

  mvprintw(
    UPPER_WIN_START_Y + 18, UPPER_WIN_START_X,
    "E : Input Call Info(ev_num, call_floor, dest_floor");
  mvprintw(
    UPPER_WIN_START_Y + 19, UPPER_WIN_START_X,
    "I : In EV");
  mvprintw(
    UPPER_WIN_START_Y + 20, UPPER_WIN_START_X,
    "DELETE : Cancel Service");
  mvprintw(
    UPPER_WIN_START_Y + 21, UPPER_WIN_START_X,
    "SPACE x 5 : Call Robot Service");
}

void MainNode::init_ev_status_window()
{
  ev_status_window_ = newwin(10, 70, 1, 1);
  keypad(ev_status_window_, TRUE);     // 특수문자 입력 가능
  box(ev_status_window_, ACS_VLINE, ACS_HLINE);
  wbkgd(ev_status_window_, COLOR_PAIR(COLOR_PAIR_WHITE_BLUE_));
  wrefresh(ev_status_window_);

  ev_status_sub_window_ = subwin(ev_status_window_, 10, 59, 1, 12);
  wbkgd(ev_status_sub_window_, COLOR_PAIR(COLOR_PAIR_WHITE_BLUE_));
  box(ev_status_sub_window_, ACS_VLINE, ACS_HLINE);
  wrefresh(ev_status_sub_window_);
}

void MainNode::init_menu_window()
{
  menu_window_ = newwin(3, 44, 12, 1);
  box(menu_window_, ACS_VLINE, ACS_HLINE);
  wbkgd(menu_window_, COLOR_PAIR(COLOR_PAIR_MENU_));

  wattron(menu_window_, COLOR_PAIR(COLOR_PAIR_MENU_ITEM_));
  mvwprintw(menu_window_, 1, 1, "ROBOT SERVICE");
  mvwprintw(menu_window_, 1, 16, "SET SEQUENCE");
  mvwprintw(menu_window_, 1, 30, "GET EV STATUS");

  wrefresh(menu_window_);

  set_menu_item_on(ROBOT_SERVICE);
}

void MainNode::init_robot_service_window()
{
  if (robot_service_window_ == nullptr) {
    robot_service_window_ = newwin(7, 30, 15, 1);
  }
  wattron(robot_service_window_, A_NORMAL);
  box(robot_service_window_, ACS_VLINE, ACS_HLINE);
  wbkgd(robot_service_window_, COLOR_PAIR(3));

  wattron(robot_service_window_, A_BOLD);
  mvwprintw(robot_service_window_, 1, 1, "EV NUM     : ");
  mvwprintw(robot_service_window_, 2, 1, "CALL FLOOR : ");
  mvwprintw(robot_service_window_, 3, 1, "DEST FLOOR : ");
  mvwprintw(robot_service_window_, 4, 1, "In EV      : ");

  wattron(robot_service_window_, A_UNDERLINE);
  wattron(robot_service_window_, COLOR_PAIR(COLOR_PAIR_ROBOT_SERVICE_DATA_));
  mvwprintw(robot_service_window_, 1, 13, "      ");
  mvwprintw(robot_service_window_, 2, 13, "      ");
  mvwprintw(robot_service_window_, 3, 13, "      ");
  mvwprintw(robot_service_window_, 4, 13, "      ");

  mvwprintw(robot_service_window_, 1, 13, call_ev_num_.c_str());
  mvwprintw(robot_service_window_, 2, 13, call_floor_.c_str());
  mvwprintw(robot_service_window_, 3, 13, dest_floor_.c_str());
  mvwprintw(robot_service_window_, 4, 13, in_ev_ ? "True" : "False");

  wrefresh(robot_service_window_);

  wattroff(robot_service_window_, A_BOLD);
  wattroff(robot_service_window_, A_UNDERLINE);
  wattroff(robot_service_window_, COLOR_PAIR(COLOR_PAIR_ROBOT_SERVICE_DATA_));

  curr_robot_service_item_ = 0;
  set_robot_service_item_on(curr_robot_service_item_);
}

void MainNode::set_robot_service_item_on(int index)
{
  wattron(robot_service_window_, A_UNDERLINE);
  wattron(robot_service_window_, COLOR_PAIR(COLOR_PAIR_ROBOT_SERVICE_DATA_));
  wattron(robot_service_window_, A_BOLD);
  mvwprintw(robot_service_window_, 1, 13, "      ");
  mvwprintw(robot_service_window_, 2, 13, "      ");
  mvwprintw(robot_service_window_, 3, 13, "      ");
  mvwprintw(robot_service_window_, 4, 13, "      ");
  mvwprintw(robot_service_window_, 1, 13, call_ev_num_.c_str());
  mvwprintw(robot_service_window_, 2, 13, call_floor_.c_str());
  mvwprintw(robot_service_window_, 3, 13, dest_floor_.c_str());
  mvwprintw(robot_service_window_, 4, 13, in_ev_ ? "True" : "False");
  wattron(robot_service_window_, COLOR_PAIR(COLOR_PAIR_ROBOT_SERVICE_ITEM_ON_));
  if (index == RobotServiceItem::EV_NUM) {
    mvwprintw(robot_service_window_, 1, 13, "      ");
    mvwprintw(robot_service_window_, 1, 13, call_ev_num_.c_str());
  } else if (index == RobotServiceItem::CALL_FLOOR) {
    mvwprintw(robot_service_window_, 2, 13, "      ");
    mvwprintw(robot_service_window_, 2, 13, call_floor_.c_str());
  } else if (index == RobotServiceItem::DEST_FLOOR) {
    mvwprintw(robot_service_window_, 3, 13, "      ");
    mvwprintw(robot_service_window_, 3, 13, dest_floor_.c_str());
  } else if (index == RobotServiceItem::IV_EV) {
    mvwprintw(robot_service_window_, 4, 13, "      ");
    mvwprintw(robot_service_window_, 4, 13, in_ev_ ? "True" : "False");
  }
  wrefresh(robot_service_window_);
  wattron(robot_service_window_, A_NORMAL);

  wattroff(robot_service_window_, A_BOLD);
  wattroff(robot_service_window_, A_UNDERLINE);
  wattroff(robot_service_window_, COLOR_PAIR(COLOR_PAIR_ROBOT_SERVICE_DATA_));
}

void MainNode::clear_robot_service_item(int index)
{
  wattron(robot_service_window_, A_UNDERLINE);
  if (index == RobotServiceItem::EV_NUM) {
    mvwprintw(robot_service_window_, 1, 13, "      ");
  } else if (index == RobotServiceItem::CALL_FLOOR) {
    mvwprintw(robot_service_window_, 2, 13, "      ");
  } else if (index == RobotServiceItem::DEST_FLOOR) {
    mvwprintw(robot_service_window_, 3, 13, "      ");
  } else if (index == RobotServiceItem::IV_EV) {
    mvwprintw(robot_service_window_, 4, 13, "      ");
  }
  wattroff(robot_service_window_, A_UNDERLINE);
  wrefresh(robot_service_window_);
}
void MainNode::init_set_sequence_window()
{
  if (set_sequence_window_ == nullptr) {
    set_sequence_window_ = newwin(7, 30, 15, 1);
  }
  box(set_sequence_window_, ACS_VLINE, ACS_HLINE);
  wbkgd(set_sequence_window_, COLOR_PAIR(3));

  wattron(set_sequence_window_, A_BOLD);
  mvwprintw(set_sequence_window_, 1, 1, "SEQUENCE    : ");
  mvwprintw(set_sequence_window_, 2, 1, "UPDATE TIME : ");
  mvwprintw(set_sequence_window_, 4, 1, "SEND TAKING_ON  ");
  mvwprintw(set_sequence_window_, 5, 1, "SEND GETTING_OFF");

  // update_sequence();

  wrefresh(set_sequence_window_);
  wattroff(set_sequence_window_, A_BOLD);

  curr_sequnce_item_ = 0;
  set_send_sequence_item_on(curr_sequnce_item_);
}

void MainNode::update_sequence()
{
  mvwprintw(set_sequence_window_, 1, 13, "                ");
  mvwprintw(set_sequence_window_, 2, 13, "                ");
  mvwprintw(set_sequence_window_, 1, 13, sequence_.c_str());
  mvwprintw(set_sequence_window_, 2, 13, topic_recv_time_.c_str());
  wrefresh(set_sequence_window_);
}

void MainNode::set_send_sequence_item_on(int index)
{
  mvwprintw(set_sequence_window_, 4, 1, "SEND TAKING_ON  ");
  mvwprintw(set_sequence_window_, 5, 1, "SEND GETTING_OFF");
  wattron(set_sequence_window_, COLOR_PAIR(COLOR_PAIR_ROBOT_SERVICE_ITEM_ON_));
  if (index == SetSequenceItem::TAKING_ON) {
    mvwprintw(set_sequence_window_, 4, 1, "SEND TAKING_ON  ");
  } else if (index == SetSequenceItem::GETTING_OFF) {
    mvwprintw(set_sequence_window_, 5, 1, "SEND GETTING_OFF");
  }
  wattroff(set_sequence_window_, COLOR_PAIR(COLOR_PAIR_ROBOT_SERVICE_ITEM_ON_));
  wrefresh(set_sequence_window_);
}

void MainNode::init_get_status_window()
{
  if (get_status_window_ == nullptr) {
    get_status_window_ = newwin(7, 30, 15, 1);
  }
  box(get_status_window_, ACS_VLINE, ACS_HLINE);
  wbkgd(get_status_window_, COLOR_PAIR(3));

  wattron(get_status_window_, A_BOLD);
  mvwprintw(get_status_window_, 1, 1, "EV_NUM    : ");
  mvwprintw(get_status_window_, 2, 1, "REPEAT    : ");
  mvwprintw(get_status_window_, 3, 1, "INTERVAL  : ");
  mvwprintw(get_status_window_, 5, 1, "GET STATUS ONCE");

  std::string get_status_interval = std::to_string(get_status_interval_sec_) + "  [s]";

  mvwprintw(get_status_window_, 1, 13, std::to_string(get_status_ev_num_).c_str());
  mvwprintw(get_status_window_, 2, 13, repeat_get_status_ ? "True" : "False");
  mvwprintw(get_status_window_, 3, 13, get_status_interval.c_str());

  wrefresh(get_status_window_);
  wattroff(get_status_window_, A_BOLD);

  curr_get_status_item_ = 0;
  set_get_status_item_on(curr_get_status_item_);
}

void MainNode::set_get_status_item_on(int index)
{
  wattron(get_status_window_, A_BOLD);

  std::string get_status_interval = std::to_string(get_status_interval_sec_) + "  [s]";

  mvwprintw(get_status_window_, 1, 13, "        ");
  mvwprintw(get_status_window_, 2, 13, "        ");
  mvwprintw(get_status_window_, 3, 13, "        ");

  mvwprintw(get_status_window_, 1, 13, std::to_string(get_status_ev_num_).c_str());
  mvwprintw(get_status_window_, 2, 13, repeat_get_status_ ? "True" : "False");
  mvwprintw(get_status_window_, 3, 13, get_status_interval.c_str());
  mvwprintw(get_status_window_, 5, 1, "GET STATUS ONCE");

  wattron(get_status_window_, COLOR_PAIR(COLOR_PAIR_ROBOT_SERVICE_ITEM_ON_));
  if (index == GetStatusItem::GET_STATUS_EV_NUM) {
    mvwprintw(get_status_window_, 1, 13, "        ");
    mvwprintw(get_status_window_, 1, 13, std::to_string(get_status_ev_num_).c_str());
  } else if (index == GetStatusItem::REPEAT) {
    mvwprintw(get_status_window_, 2, 13, "        ");
    mvwprintw(get_status_window_, 2, 13, repeat_get_status_ ? "True" : "False");
  } else if (index == GetStatusItem::INTERVAL) {
    mvwprintw(get_status_window_, 3, 13, "        ");
    mvwprintw(get_status_window_, 3, 13, get_status_interval.c_str());
  } else if (index == GetStatusItem::GET_STATUS_ONCE) {
    mvwprintw(get_status_window_, 5, 1, "GET STATUS ONCE");
  }
  wattroff(get_status_window_, COLOR_PAIR(COLOR_PAIR_ROBOT_SERVICE_ITEM_ON_));
  wattroff(get_status_window_, A_BOLD);
  wrefresh(get_status_window_);
}

void MainNode::clear_get_status_item(int index)
{
  wattron(get_status_window_, A_UNDERLINE);
  if (index == GetStatusItem::GET_STATUS_EV_NUM) {
    mvwprintw(get_status_window_, 1, 13, "        ");
  } else if (index == GetStatusItem::REPEAT) {
    mvwprintw(get_status_window_, 2, 13, "        ");
  } else if (index == GetStatusItem::INTERVAL) {
    mvwprintw(get_status_window_, 3, 13, "        ");
  }
  wattroff(get_status_window_, A_UNDERLINE);
  wrefresh(get_status_window_);
}

void MainNode::set_menu_item_on(int index)
{
  wattron(menu_window_, COLOR_PAIR(COLOR_PAIR_MENU_ITEM_));
  mvwprintw(menu_window_, 1, 1, "ROBOT SERVICE");
  mvwprintw(menu_window_, 1, 16, "SET SEQUENCE");
  mvwprintw(menu_window_, 1, 30, "GET EV STATUS");
  wattron(menu_window_, COLOR_PAIR(COLOR_PAIR_MENU_ITEM_ON_));

  werase(robot_service_window_);
  wclear(robot_service_window_);

  werase(set_sequence_window_);
  wclear(set_sequence_window_);

  wrefresh(robot_service_window_);
  wrefresh(set_sequence_window_);

  if (index == ROBOT_SERVICE) {
    mvwprintw(menu_window_, 1, 1, "ROBOT SERVICE");
    touchwin(robot_service_window_);
    init_robot_service_window();
  } else if (index == SET_SEQUENCE) {
    mvwprintw(menu_window_, 1, 16, "SET SEQUENCE");
    touchwin(set_sequence_window_);
    init_set_sequence_window();
  } else if (index == GET_EV_STATUS) {
    mvwprintw(menu_window_, 1, 30, "GET EV STATUS");
    touchwin(get_status_window_);
    init_get_status_window();
  }
  touchwin(stdscr);
  wrefresh(menu_window_);
}


void MainNode::print_test(std::string value)
{
  print_ev_num("ev_num_1, ev_num_2");
  print_ev_name("ev_name_1, ev_name_2");
  print_floor("B1, 21");
  print_direction("Up, None");
  print_run("Running, Stoped");
  print_door("Opened, Closed");
  print_mode("IDLE, ERROR");
  print_in_ev(in_ev_);
}

void MainNode::print_ev_status()
{
  print_ev_num(ev_num_);
  print_ev_name(ev_name_);
  print_floor(floor_);
  print_direction(direction_);
  print_run(run_);
  print_door(door_);
  print_mode(mode_);
  wrefresh(ev_status_sub_window_);
}

void MainNode::init_color_pair()
{
  init_pair(COLOR_PAIR_BG, COLOR_BLACK, COLOR_BLACK);
  init_pair(COLOR_PAIR_WHITE_BLUE_, COLOR_WHITE, COLOR_BLUE);
  init_pair(COLOR_PAIR_BLUE_GREEN_, COLOR_BLUE, COLOR_GREEN);
  init_pair(COLOR_PAIR_BLACK_GREEN_, COLOR_BLACK, COLOR_GREEN);
  init_pair(COLOR_PAIR_MENU_, COLOR_RED, COLOR_BLACK);
  init_pair(COLOR_PAIR_MENU_ITEM_, COLOR_BLACK, COLOR_WHITE);
  init_pair(COLOR_PAIR_MENU_ITEM_ON_, COLOR_RED, COLOR_YELLOW);
  init_pair(COLOR_PAIR_ROBOT_SERVICE_DATA_, COLOR_BLACK, COLOR_WHITE);
  init_pair(COLOR_PAIR_ROBOT_SERVICE_ITEM_ON_, COLOR_BLACK, COLOR_YELLOW);

}

void MainNode::print_ev_num(std::string value)
{
  mvwprintw(
    ev_status_sub_window_, UPPER_WIN_START_Y + 1, 1,
    "                                                        ");
  mvwprintw(ev_status_sub_window_, UPPER_WIN_START_Y + 1, 1, value.c_str());
}

void MainNode::print_ev_name(std::string value)
{
  mvwprintw(
    ev_status_sub_window_, UPPER_WIN_START_Y + 2, 1,
    "                                                        ");
  mvwprintw(ev_status_sub_window_, UPPER_WIN_START_Y + 2, 1, value.c_str());
}

void MainNode::print_floor(std::string value)
{
  mvwprintw(
    ev_status_sub_window_, UPPER_WIN_START_Y + 3, 1,
    "                                                        ");
  mvwprintw(ev_status_sub_window_, UPPER_WIN_START_Y + 3, 1, value.c_str());
}


void MainNode::print_direction(std::string value)
{
  mvwprintw(
    ev_status_sub_window_, UPPER_WIN_START_Y + 4, 1,
    "                                                        ");
  mvwprintw(ev_status_sub_window_, UPPER_WIN_START_Y + 4, 1, value.c_str());
}

void MainNode::print_run(std::string value)
{
  mvwprintw(
    ev_status_sub_window_, UPPER_WIN_START_Y + 5, 1,
    "                                                        ");
  mvwprintw(ev_status_sub_window_, UPPER_WIN_START_Y + 5, 1, value.c_str());
}


void MainNode::print_door(std::string value)
{
  mvwprintw(
    ev_status_sub_window_, UPPER_WIN_START_Y + 6, 1,
    "                                                        ");
  mvwprintw(ev_status_sub_window_, UPPER_WIN_START_Y + 6, 1, value.c_str());
}

void MainNode::print_mode(std::string value)
{
  mvwprintw(
    ev_status_sub_window_, UPPER_WIN_START_Y + 7, 1,
    "                                                        ");
  mvwprintw(ev_status_sub_window_, UPPER_WIN_START_Y + 7, 1, value.c_str());
}

void MainNode::print_call_ev_num(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 9, UPPER_WIN_START_X + 13, value.c_str());
}

void MainNode::print_call_floor(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 10, UPPER_WIN_START_X + 13, value.c_str());
}

void MainNode::print_dest_floor(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 11, UPPER_WIN_START_X + 13, value.c_str());
}

void MainNode::print_in_ev(bool on)
{
  if (on) {
    mvprintw(UPPER_WIN_START_Y + 12, UPPER_WIN_START_X + 14, "V");
  } else {
    mvprintw(UPPER_WIN_START_Y + 12, UPPER_WIN_START_X + 14, " ");
  }
}

void MainNode::print_sequence(std::string value)
{
  // attron(A_BLINK);
  mvprintw(
    UPPER_WIN_START_Y + 13, UPPER_WIN_START_X + 13,
    "                                             ");
  mvprintw(
    UPPER_WIN_START_Y + 13, UPPER_WIN_START_X + 13,
    (value + "  /  " + topic_recv_time_).c_str());
  // mvprintw(UPPER_WIN_START_Y + 13, UPPER_WIN_START_X + 25, topic_recv_time_.c_str());
  //attroff(A_BLINK);
}

void MainNode::print_service_result(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 14, UPPER_WIN_START_X + 13, "                         ");
  mvprintw(UPPER_WIN_START_Y + 14, UPPER_WIN_START_X + 13, value.c_str());
}

void MainNode::input_handler(int c)
{
  if (c == '\n') {  // Enter
    // this->print_test("");
    // get_ev_status(0);
    // cnt_ = 0;
    if (curr_menu_index_ == MenuItem::ROBOT_SERVICE) {
      curs_set(1);     // 커서 보이게
      echo();     // 입력문자 보이게
      char input[10];
      clear_robot_service_item(curr_robot_service_item_);
      if (curr_robot_service_item_ == RobotServiceItem::EV_NUM) {
        wmove(robot_service_window_, 1, 13);
        wgetstr(robot_service_window_, input);
        if (std::string(input) != "") {
          call_ev_num_ = std::string(input);
        }
      } else if (curr_robot_service_item_ == RobotServiceItem::CALL_FLOOR) {
        wmove(robot_service_window_, 2, 13);
        wgetstr(robot_service_window_, input);
        if (std::string(input) != "") {
          call_floor_ = std::string(input);
        }
      } else if (curr_robot_service_item_ == RobotServiceItem::DEST_FLOOR) {
        wmove(robot_service_window_, 3, 13);
        wgetstr(robot_service_window_, input);
        if (std::string(input) != "") {
          dest_floor_ = std::string(input);
        }
      } else if (curr_robot_service_item_ == RobotServiceItem::IV_EV) {
        wmove(robot_service_window_, 4, 13);
        if (std::string(input) != "") {
          wgetstr(robot_service_window_, input);
        }
        in_ev_ = in_ev_ ? false : true;
      }
      set_robot_service_item_on(curr_robot_service_item_);
      curs_set(0);  // 커서 안보이게
      noecho();  // 입력 문자 안보이게
    } else if (curr_menu_index_ == MenuItem::SET_SEQUENCE) {
      if (curr_sequnce_item_ == SetSequenceItem::TAKING_ON) {
      } else if (curr_sequnce_item_ == SetSequenceItem::GETTING_OFF) {
      }
    } else if (curr_menu_index_ == MenuItem::GET_EV_STATUS) {
      if (curr_get_status_item_ == GetStatusItem::GET_STATUS_EV_NUM) {
        curs_set(1);   // 커서 보이게
        echo();   // 입력문자 보이게
        char input[10];
        clear_get_status_item(curr_get_status_item_);

        wmove(get_status_window_, 1, 13);
        wgetstr(get_status_window_, input);

        if (std::string(input) != "") {
          try {
            get_status_ev_num_ = std::stoi(input);
          } catch (...) {
          }
        }
        curs_set(0); // 커서 안보이게
        noecho(); // 입력 문자 안보이게
      } else if (curr_get_status_item_ == GetStatusItem::REPEAT) {
        repeat_get_status_ = repeat_get_status_ ? false : true;
      } else if (curr_get_status_item_ == GetStatusItem::INTERVAL) {
        curs_set(1);   // 커서 보이게
        echo();   // 입력문자 보이게
        char input[10];
        clear_get_status_item(curr_get_status_item_);

        wmove(get_status_window_, 3, 13);
        wgetstr(get_status_window_, input);

        if (std::string(input) != "") {
          try {
            get_status_interval_sec_ = std::stoi(input);
          } catch (...) {
          }
        }
        curs_set(0); // 커서 안보이게
        noecho(); // 입력 문자 안보이게
      } else if (curr_get_status_item_ == GetStatusItem::GET_STATUS_ONCE) {
        get_ev_status(get_status_ev_num_);
      }
      set_get_status_item_on(curr_get_status_item_);
    }
  } else if (c == 'i' || c == 'I') {
    in_ev_ = in_ev_ ? false : true;
    print_in_ev(in_ev_);
    cnt_ = 0;
  } else if (c == 'e' || c == 'E') {

    sub_window = newwin(10, 30, 5, 5);
    box(sub_window, ACS_VLINE, ACS_HLINE);
    wmove(sub_window, 2, 2);
    mvwprintw(sub_window, 2, 2, "EV Num     : ");
    mvwprintw(sub_window, 3, 2, "Call Floor : ");
    mvwprintw(sub_window, 4, 2, "Dest Floor : ");
    wmove(sub_window, 2, 15);
    curs_set(1);     // 커서 보이게
    echo();     // 입력문자 보이게
    wbkgd(sub_window, COLOR_PAIR(3));
    refresh();
    char input[10];
    wgetstr(sub_window, input);
    call_ev_num_ = std::string(input);

    wmove(sub_window, 3, 15);
    wgetstr(sub_window, input);
    call_floor_ = std::string(input);

    wmove(sub_window, 4, 15);
    wgetstr(sub_window, input);
    dest_floor_ = std::string(input);

    print_call_ev_num(call_ev_num_);
    print_call_floor(call_floor_);
    print_dest_floor(dest_floor_);
    touchwin(stdscr);
    werase(sub_window);
    wclear(sub_window);
    delwin(sub_window);
    curs_set(0);  // 커서 안보이게
    noecho();  // 입력 문자 안보이게
    refresh();
    cnt_ = 0;
  } else if (c == 27) {    // ESC
    cancel_robot_service();
    cnt_ = 0;
  } else if (c == 't' || c == 'T') {
    set_robot_service("Taking On");
    cnt_ = 0;
  } else if (c == 'g' || c == 'G') {
    set_robot_service("Getting Off");
    cnt_ = 0;
  } else if (c == 'r' || c == 'R') {
    clear();
    endwin();
    init_window();
  } else if (c == KEY_RIGHT || c == KEY_LEFT) {
    if (c == KEY_RIGHT) {
      curr_menu_index_++;
    } else {
      curr_menu_index_--;
    }
    if (curr_menu_index_ > 2) {
      curr_menu_index_ = 0;
    } else if (curr_menu_index_ < 0) {
      curr_menu_index_ = 2;
    }
    set_menu_item_on(curr_menu_index_);
  } else if (c == KEY_UP || c == KEY_DOWN) {
    if (curr_menu_index_ == MenuItem::ROBOT_SERVICE) {
      if (c == KEY_DOWN) {
        curr_robot_service_item_++;
      } else {
        curr_robot_service_item_--;
      }
      if (curr_robot_service_item_ > 3) {
        curr_robot_service_item_ = 0;
      } else if (curr_robot_service_item_ < 0) {
        curr_robot_service_item_ = 3;
      }
      set_robot_service_item_on(curr_robot_service_item_);
    } else if (curr_menu_index_ == MenuItem::SET_SEQUENCE) {
      curr_sequnce_item_ = curr_sequnce_item_ == 0 ? 1 : 0;
      set_send_sequence_item_on(curr_sequnce_item_);
    } else if (curr_menu_index_ == MenuItem::GET_EV_STATUS) {
      if (c == KEY_DOWN) {
        curr_get_status_item_++;
      } else {
        curr_get_status_item_--;
      }
      if (curr_get_status_item_ > 3) {
        curr_get_status_item_ = 0;
      } else if (curr_get_status_item_ < 0) {
        curr_get_status_item_ = 3;
      }
      set_get_status_item_on(curr_get_status_item_);
    }
  }

  if (c == ' ') {
    if (++cnt_ > 4) {
      int ev_num;
      try {
        ev_num = std::stoi(call_ev_num_);
        if (in_ev_) {
          robot_service_in_ev_call(ev_num, dest_floor_);
        } else {
          robot_service_call(ev_num, call_floor_, dest_floor_);
        }
        print_service_result(robot_service_result_);
      } catch (...) {
        robot_service_result_ = "False";
        print_service_result(robot_service_result_);
      }
      cnt_ = 0;
    }
  }
}

void MainNode::get_keyboard_input(int & c)
{
  c = wgetch(ev_status_window_);
}

int main(int argc, char * argv[])
{
  //setlocale(LC_ALL, "ko_KR.utf8");

  // setlocale(LC_ALL, "");
  // setlocale(LC_CTYPE, "");
  //setlocale(LC_CTYPE, "ko_KR.eucKR");


  // setlocale(LC_ALL, "EUC-KR");
  //  setlocale(LC_CTYPE, "EUC-KR");

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MainNode>();

  std::thread spin_thread;

  auto ros_spin = [node]() {
      rclcpp::executors::SingleThreadedExecutor exec;
      rclcpp::WallRate loop_rate(10);

      while (rclcpp::ok()) {
        exec.spin_node_some(node);
        loop_rate.sleep();
      }
      return;
    };
  spin_thread = std::thread(ros_spin);

  int c;
  rclcpp::WallRate loop_rate(10);

  while (rclcpp::ok()) {
    nodelay(stdscr, TRUE);
    node->get_keyboard_input(c);

    if (c == 'q' || c == 'Q') {
      endwin();
      break;
    }

    node->input_handler(c);

//    rclcpp::spin_some(node);

    loop_rate.sleep();
  }
  // rclcpp::spin(node);

  rclcpp::shutdown();
  endwin();
  return 0;
}
