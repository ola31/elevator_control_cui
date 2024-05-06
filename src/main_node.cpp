#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <string>
#include <memory>
#include <vector>
// #include <ncurses.h>
#include <locale.h>

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
  delwin(sub_window);
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

      print_ev_num(ev_num_);
      print_ev_name(ev_name_);
      print_floor(floor_);
      print_direction(direction_);
      print_run(run_);
      print_door(door_);
      print_mode(mode_);
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

  init_window_format();
  print_ev_status();
}

void MainNode::init_window_format()
{
  mvprintw(UPPER_WIN_START_Y + 0, UPPER_WIN_START_X, "----------------------------");
  mvprintw(UPPER_WIN_START_Y + 1, UPPER_WIN_START_X, "ev_num    : ");
  mvprintw(UPPER_WIN_START_Y + 2, UPPER_WIN_START_X, "ev_name   : ");
  mvprintw(UPPER_WIN_START_Y + 3, UPPER_WIN_START_X, "floor     : ");
  mvprintw(UPPER_WIN_START_Y + 4, UPPER_WIN_START_X, "direction : ");
  mvprintw(UPPER_WIN_START_Y + 5, UPPER_WIN_START_X, "run       : ");
  mvprintw(UPPER_WIN_START_Y + 6, UPPER_WIN_START_X, "door      : ");
  mvprintw(UPPER_WIN_START_Y + 7, UPPER_WIN_START_X, "mode      : ");
  mvprintw(UPPER_WIN_START_Y + 8, UPPER_WIN_START_X, "----------------------------");
  mvprintw(UPPER_WIN_START_Y + 9, UPPER_WIN_START_X, "ev_num     : ");
  mvprintw(UPPER_WIN_START_Y + 10, UPPER_WIN_START_X, "call_floor : ");
  mvprintw(UPPER_WIN_START_Y + 11, UPPER_WIN_START_X, "dest_floor : ");
  mvprintw(UPPER_WIN_START_Y + 12, UPPER_WIN_START_X, "In EV      : [ ]");
  mvprintw(UPPER_WIN_START_Y + 13, UPPER_WIN_START_X, "Sequence   : ");
  mvprintw(UPPER_WIN_START_Y + 14, UPPER_WIN_START_X, "result     : ");
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
  print_in_ev(in_ev_);
}

void MainNode::print_ev_num(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 1, UPPER_WIN_START_X + 12, value.c_str());
}

void MainNode::print_ev_name(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 2, UPPER_WIN_START_X + 12, value.c_str());
}

void MainNode::print_floor(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 3, UPPER_WIN_START_X + 12, "                         ");
  mvprintw(UPPER_WIN_START_Y + 3, UPPER_WIN_START_X + 12, value.c_str());
}


void MainNode::print_direction(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 4, UPPER_WIN_START_X + 12, "                         ");
  mvprintw(UPPER_WIN_START_Y + 4, UPPER_WIN_START_X + 12, value.c_str());
}

void MainNode::print_run(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 5, UPPER_WIN_START_X + 12, "                         ");
  mvprintw(UPPER_WIN_START_Y + 5, UPPER_WIN_START_X + 12, value.c_str());
}


void MainNode::print_door(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 6, UPPER_WIN_START_X + 12, "                         ");
  mvprintw(UPPER_WIN_START_Y + 6, UPPER_WIN_START_X + 12, value.c_str());
}

void MainNode::print_mode(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 7, UPPER_WIN_START_X + 12, "                         ");
  mvprintw(UPPER_WIN_START_Y + 7, UPPER_WIN_START_X + 12, value.c_str());
}

void MainNode::print_call_ev_num(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 9, UPPER_WIN_START_X + 12, "                         ");
  mvprintw(UPPER_WIN_START_Y + 9, UPPER_WIN_START_X + 13, value.c_str());
}

void MainNode::print_call_floor(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 10, UPPER_WIN_START_X + 12, "                         ");
  mvprintw(UPPER_WIN_START_Y + 10, UPPER_WIN_START_X + 13, value.c_str());
}

void MainNode::print_dest_floor(std::string value)
{
  mvprintw(UPPER_WIN_START_Y + 11, UPPER_WIN_START_X + 12, "                         ");
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

void MainNode::input_handler(char c)
{
  if (c == '\n') {  // Enter
    // this->print_test("");
    get_ev_status(0);
    cnt_ = 0;
  } else if (c == 'i' || c == 'I') {
    in_ev_ = in_ev_ ? false : true;
    print_in_ev(in_ev_);
    cnt_ = 0;
  } else if (c == 'e' || c == 'E') {

    WINDOW * sub_window = newwin(10, 30, 5, 5);
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
  } else if (c == 'c' || c == 'C') {
    WINDOW * sub_window = newwin(5, 30, 5, 5);
    box(sub_window, ACS_VLINE, ACS_HLINE);
    wmove(sub_window, 2, 2);
    wprintw(sub_window, "Call Floor : ");
    curs_set(1);     // 커서 보이게
    echo();     // 입력문자 보이게
    wbkgd(sub_window, COLOR_PAIR(3));
    refresh();
    char floor[10];
    wgetstr(sub_window, floor);
    call_floor_ = std::string(floor);
    print_call_floor(call_floor_);
    touchwin(stdscr);
    werase(sub_window);
    wclear(sub_window);
    delwin(sub_window);
    curs_set(0);  // 커서 안보이게
    noecho();  // 입력 문자 안보이게
    refresh();
    cnt_ = 0;
  } else if (c == 'd' || c == 'D') {
    WINDOW * sub_window = newwin(5, 30, 5, 5);
    box(sub_window, ACS_VLINE, ACS_HLINE);
    wmove(sub_window, 2, 2);
    wprintw(sub_window, "Dest Floor : ");
    curs_set(1);       // 커서 보이게
    echo();       // 입력문자 보이게
    wbkgd(sub_window, COLOR_PAIR(3));
    refresh();
    char floor[10];
    wgetstr(sub_window, floor);
    dest_floor_ = std::string(floor);
    print_dest_floor(dest_floor_);
    touchwin(stdscr);
    werase(sub_window);
    wclear(sub_window);
    delwin(sub_window);
    curs_set(0);    // 커서 안보이게
    noecho();    // 입력 문자 안보이게
    refresh();
    cnt_ = 0;
  } else if (c == 'e' || c == 'E') {
    WINDOW * sub_window = newwin(5, 30, 5, 5);
    box(sub_window, ACS_VLINE, ACS_HLINE);
    wmove(sub_window, 2, 2);
    wprintw(sub_window, "ev_num : ");
    curs_set(1);       // 커서 보이게
    echo();       // 입력문자 보이게
    wbkgd(sub_window, COLOR_PAIR(3));
    refresh();
    char input[10];
    wgetstr(sub_window, input);
    call_ev_num_ = std::string(input);
    print_call_ev_num(call_ev_num_);
    touchwin(stdscr);
    werase(sub_window);
    wclear(sub_window);
    delwin(sub_window);
    curs_set(0);    // 커서 안보이게
    noecho();    // 입력 문자 안보이게
    refresh();
    cnt_ = 0;
  } else if (c == 27) {  // ESC
    cancel_robot_service();
    cnt_ = 0;
  } else if (c == 't' || c == 'T') {
    set_robot_service("Taking On");
    cnt_ = 0;
  } else if (c == 'g' || c == 'G') {
    set_robot_service("Getting Off");
    cnt_ = 0;
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

int main(int argc, char * argv[])
{
  //setlocale(LC_ALL, "ko_KR.utf8");

  // setlocale(LC_ALL, "");
  // setlocale(LC_CTYPE, "");
  //setlocale(LC_CTYPE, "ko_KR.eucKR");


  setlocale(LC_ALL, "EUC-KR");
  setlocale(LC_CTYPE, "EUC-KR");

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MainNode>();

  rclcpp::WallRate loop_rate(10);

  usleep(1000000);  // 1 sec

  char c;
  char str[20];

  while (rclcpp::ok()) {
    nodelay(stdscr, TRUE);
    c = getch();

    getstr(str);

    if (c == 'q' || c == 'Q') {
      break;
    }

    node->input_handler(c);

    refresh();

    rclcpp::spin_some(node);

    loop_rate.sleep();
  }
  // rclcpp::spin(node);

  rclcpp::shutdown();
  endwin();
  return 0;
}
