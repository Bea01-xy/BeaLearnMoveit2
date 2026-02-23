#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

// 定义一个类来管理 MoveIt 逻辑
class MoveItPlanner {
public:
    MoveItPlanner(const std::shared_ptr<rclcpp::Node>& node)
        : node_(node) {

        // 1. 初始化 MoveGroupInterface (针对 panda_arm)
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "panda_arm");

        // 2. 初始化可视化工具
        moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
            move_group_interface_->getRobotModel());

        moveit_visual_tools_->deleteAllMarkers();
        moveit_visual_tools_->loadRemoteControl();

        // 3. 声明 ROS 2 参数，允许在运行期间通过 rqt 修改
        node_->declare_parameter("target_x", 0.1);
        node_->declare_parameter("target_y", 0.4);
        node_->declare_parameter("target_z", 0.4);
        node_->declare_parameter("target_ox", 0.0);
        node_->declare_parameter("target_oy", 0.8);
        node_->declare_parameter("target_oz", 0.0);
        node_->declare_parameter("target_ow", 0.6);

        // 4. 创建服务回调：当调用该服务时，执行规划与移动
        trigger_service_ = node_->create_service<std_srvs::srv::Trigger>(
            "plan_and_move",
            std::bind(&MoveItPlanner::handle_trigger, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(node_->get_logger(), "MoveIt 交互节点已启动。");
        RCLCPP_INFO(node_->get_logger(), "请在 rqt 中设置参数，然后调用 /plan_and_move 服务。");
    }

private:
    void handle_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request; // 忽略未使用的参数

        // A. 从参数服务器获取最新位姿
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = node_->get_parameter("target_x").as_double();
        target_pose.position.y = node_->get_parameter("target_y").as_double();
        target_pose.position.z = node_->get_parameter("target_z").as_double();
        target_pose.orientation.x = node_->get_parameter("target_ox").as_double();
        target_pose.orientation.y = node_->get_parameter("target_oy").as_double();
        target_pose.orientation.z = node_->get_parameter("target_oz").as_double();
        target_pose.orientation.w = node_->get_parameter("target_ow").as_double();

        RCLCPP_INFO(node_->get_logger(), "收到触发请求，目标位置: [x:%.2f, y:%.2f, z:%.2f]", 
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);

        // B. 设置目标并规划
        move_group_interface_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (static_cast<bool>(move_group_interface_->plan(plan)));

        if (success) {
            draw_title("Planning Success");
            moveit_visual_tools_->publishTrajectoryLine(plan.trajectory_, move_group_interface_->getRobotModel()->getJointModelGroup("panda_arm"));
            moveit_visual_tools_->trigger();

            // C. 执行规划
            draw_title("Executing...");
            moveit_visual_tools_->trigger();
            move_group_interface_->execute(plan);

            response->success = true;
            response->message = "规划并执行成功";
        } else {
            draw_title("Planning Failed!");
            moveit_visual_tools_->trigger();
            RCLCPP_ERROR(node_->get_logger(), "规划失败！");
            response->success = false;
            response->message = "规划失败，请检查位姿是否可达";
        }
    }

    void draw_title(const std::string& text) {
        auto const text_pose = [] {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.2; 
            return msg;
        }();
        moveit_visual_tools_->publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // 使用 NodeOptions 允许参数覆盖
    auto const node = std::make_shared<rclcpp::Node>(
        "interactive_moveit_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // 为了让 MoveIt 在后台处理回调，需要使用多线程执行器
    // 否则服务回调可能会与 MoveGroup 的内部机制产生死锁
    rclcpp::executors::MultiThreadedExecutor executor;
    auto planner = std::make_shared<MoveItPlanner>(node);

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
