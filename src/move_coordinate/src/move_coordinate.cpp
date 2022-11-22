#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move_coordinate", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  //로봇의 상태 정보 획득을 위한 SingleThreadedExecutor 가동
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  static const std::string PLANNING_GROUP = "panda_arm";

  // ROS logger 생성 (ROS관련 에러 메시지 출력하는데 쓰임)
  auto const logger = rclcpp::get_logger("move_coordinate");

  // MoveIt MoveGroup Interface 생성
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);

  // 로봇의 현재 상태 생성 (각,속도,돌림힘의 현재 상태 출력하는데 필요)
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
  std::vector<double> joint_group_positions;
  const moveit::core::JointModelGroup *joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  while (true)
  {
    char c;
    std::cout << "\n위치 초기화: R / 좌표 입력: I / 관절각도 입력: O / 프로그램 종료: Q" << std::endl;

    std::cin.clear();
    std::cin >> c;

    if (c == 'r' || c == 'R')
    {

      joint_group_positions[0] = 0.0;
      joint_group_positions[1] = -0.8;
      joint_group_positions[2] = 0.0;
      joint_group_positions[3] = -2.4;
      joint_group_positions[4] = 0.0;
      joint_group_positions[5] = 1.6;
      joint_group_positions[6] = 0.8;
      move_group_interface.setJointValueTarget(joint_group_positions);

      auto const [success, plan] = [&move_group_interface]
      {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();
      // 경로 실행
      if (success)
      {
        move_group_interface.execute(plan);
      }
      else
      {
        RCLCPP_ERROR(logger, "Planning failed!");
      }
    }
    else if (c == 'i' || c == 'I')
    {

      // 목표 좌표 세팅
      auto const target_pose = []
      {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        std::cout << "좌표 입력 :";
        std::cin.clear();
        std::cin >> msg.position.x >> msg.position.y >> msg.position.z;
        return msg;
      }();
      //해당 좌표에 대한 관절 각도 출력
      for (int i = 0; i < joint_group_positions.size(); i++)
      {
        printf("joint[%d] : %0.1lf\n", i, joint_group_positions[i]);
      }
      move_group_interface.setPoseTarget(target_pose);

      // 목표 좌표에 대한 경로 생성
      auto const [success, plan] = [&move_group_interface]
      {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();
      // 경로 실행
      if (success)
      {
        move_group_interface.execute(plan);
      }
      else
      {
        RCLCPP_ERROR(logger, "Planning failed!");
      }
    }
    else if (c == 'o' || c == 'O')
    {
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 0.1;
      // joint_group_positions[0] = -1.0; // radians
      move_group_interface.setJointValueTarget(joint_group_positions);

      for (int i = 0; i < joint_group_positions.size(); i++)
      {
        printf("joint[%d] : %0.1lf\n", i, joint_group_positions[i]);
      }

      auto const [success, plan] = [&move_group_interface]
      {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();
      // 경로 실행
      if (success)
      {
        move_group_interface.execute(plan);
      }
      else
      {
        RCLCPP_ERROR(logger, "Planning failed!");
      }
    }
    else if (c == 'q' || c == 'Q')
    {
      break; // q
    }
  }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}