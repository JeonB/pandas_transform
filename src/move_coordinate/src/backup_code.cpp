#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "move_coordinate", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("move_coordinate");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "panda_arm");
    char c;
    while (true)
    {
        std::cout << "\n좌표 입력: R / 관절각도 입력: T / 프로그램 종료: Q" << std::endl;

        std::cin.clear();
        std::cin >> c;
        if (c == 'q' || c == 'Q')
        {
            break; // q
        }
        else if (c == 't' || c == 'T')
        {
            printf("test");
        }
        else if (c == 'r' || c == 'R')
        {

            // Set a target Pose
            auto const target_pose = []
            {
                geometry_msgs::msg::Pose msg;
                msg.orientation.w = 1.0;
                std::cout << "좌표 입력 :";
                std::cin.clear();
                std::cin >> msg.position.x >> msg.position.y >> msg.position.z;
                return msg;
            }();
            move_group_interface.setPoseTarget(target_pose);

            // Create a plan to that target pose
            auto const [success, plan] = [&move_group_interface]
            {
                moveit::planning_interface::MoveGroupInterface::Plan msg;
                auto const ok = static_cast<bool>(move_group_interface.plan(msg));
                return std::make_pair(ok, msg);
            }();
            // Execute the plan
            if (success)
            {
                move_group_interface.execute(plan);
            }
            else
            {
                RCLCPP_ERROR(logger, "Planning failed!");
            }
        }
    }
    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}