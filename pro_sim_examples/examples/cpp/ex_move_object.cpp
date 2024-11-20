/// Example that uses MoveIt 2 to follow a target inside Ignition Gazebo

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

const std::string MOVE_GROUP = "pro_arm";

class MoveItMoveObject : public rclcpp::Node
{
public:
    MoveItMoveObject();
    void addCollisionObjects();
    void moveToObject();
    void attachObject();
    void moveToTarget();
    void dettachObject();
    void moveToHome();

private:
    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    moveit_msgs::msg::CollisionObject object_;
};

MoveItMoveObject::MoveItMoveObject() : Node("ex_move_object"), move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
{
    addCollisionObjects();

    rclcpp::sleep_for(std::chrono::seconds(10));

    this->move_group_.setMaxAccelerationScalingFactor(0.5);
    this->move_group_.setMaxVelocityScalingFactor(0.5);
}

void MoveItMoveObject::addCollisionObjects()
{
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table_1";
  collision_objects[0].header.frame_id = "pro_arm_base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.15;
  collision_objects[0].primitives[0].dimensions[1] = 0.25;
  collision_objects[0].primitives[0].dimensions[2] = 0.55;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.35;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table_2";
  collision_objects[1].header.frame_id = "pro_arm_base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.25;
  collision_objects[1].primitives[0].dimensions[1] = 0.15;
  collision_objects[1].primitives[0].dimensions[2] = 0.49;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.35;
  collision_objects[1].primitive_poses[0].position.z = 0;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "pro_arm_base_link";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.05;
  collision_objects[2].primitives[0].dimensions[1] = 0.05;
  collision_objects[2].primitives[0].dimensions[2] = 0.05;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.3;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.3;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;

  collision_objects[2].operation = collision_objects[2].ADD;

  // Store the object for later use
  object_ = collision_objects[2];

  planning_scene_interface_.applyCollisionObjects(collision_objects);
}

void MoveItMoveObject::moveToObject()
{
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.frame_id = "pro_arm_base_link";
    start_pose.pose.position.x = 0.3;
    start_pose.pose.position.y = 0;
    start_pose.pose.position.z = 0.325;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, 0); // Roll, Pitch, and Yaw

    start_pose.pose.orientation.w = q.w();
    start_pose.pose.orientation.x = q.x();
    start_pose.pose.orientation.y = q.y();
    start_pose.pose.orientation.z = q.z();

    this->move_group_.setPoseTarget(start_pose.pose);
    this->move_group_.move();

    RCLCPP_INFO(this->get_logger(), "Moving to object");
    rclcpp::sleep_for(std::chrono::seconds(8));
}

void MoveItMoveObject::attachObject()
{
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "pro_arm_ee";
    attached_object.object = object_;

    planning_scene_interface_.applyAttachedCollisionObject(attached_object);
    RCLCPP_INFO(this->get_logger(), "Attached the object");
}

void MoveItMoveObject::moveToTarget()
{
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "pro_arm_base_link";
    target_pose.pose.position.x = 0;
    target_pose.pose.position.y = 0.3;
    target_pose.pose.position.z = 0.295;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0); // Roll, Pitch, and Yaw

    target_pose.pose.orientation.w = q.w();
    target_pose.pose.orientation.x = q.x();
    target_pose.pose.orientation.y = q.y();
    target_pose.pose.orientation.z = q.z();

    this->move_group_.setPoseTarget(target_pose.pose);
    this->move_group_.move();

    RCLCPP_INFO(this->get_logger(), "Moving to target position");
    rclcpp::sleep_for(std::chrono::seconds(12));
}

void MoveItMoveObject::dettachObject()
{
    moveit_msgs::msg::AttachedCollisionObject detach_object;
    detach_object.link_name = "pro_arm_ee";
    detach_object.object = object_;
    detach_object.object.operation = detach_object.object.REMOVE;
    planning_scene_interface_.applyAttachedCollisionObject(detach_object);
    RCLCPP_INFO(this->get_logger(), "Detached the object");
}

void MoveItMoveObject::moveToHome()
{
    move_group_.setNamedTarget("default");
    this->move_group_.move();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto move_object = std::make_shared<MoveItMoveObject>();

    move_object->moveToObject();
    move_object->attachObject();
    move_object->moveToTarget();
    move_object->dettachObject();
    move_object->moveToHome();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_object);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}