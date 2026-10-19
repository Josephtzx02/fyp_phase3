#include <vector>
#include <memory>
#include <cmath>
#include <Eigen/Geometry>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tm5_book_msgs/msg/grasp_target.hpp>
#include <tm5_book_msgs/msg/mission.hpp>

class TM5MoveItBridge : public rclcpp::Node
{
public:
    TM5MoveItBridge()
        : Node("tm5_moveit_bridge")
    {
        subscription_ = this->create_subscription<
            tm5_book_msgs::msg::GraspTarget>(
            "/tm5/grasp_target",
            10,
            std::bind(
                &TM5MoveItBridge::target_callback,
                this,
                std::placeholders::_1
            )
        );

    	mission_subscription_ = this->create_subscription<
            tm5_book_msgs::msg::Mission>(
            "/tm5/mission",
            10,
            std::bind(
            	&TM5MoveItBridge::mission_callback,
            	this,
            	std::placeholders::_1
            )
    	);

        home_subscription_ = this->create_subscription<
            std_msgs::msg::String>(
            "/tm5/home",
            10,
            std::bind(
                &TM5MoveItBridge::home_callback,
                this,
                std::placeholders::_1
            )
        );

        status_publisher_ = this->create_publisher<
            std_msgs::msg::String>(
            "/tm5/status",
            10
        );
        
        gripper_command_publisher_ = this->create_publisher<
            std_msgs::msg::String>(
            "/tm5/gripper_command",
            10
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Step 19C-5: GraspTarget -> MoveIt bridge"
        );
    }

    void set_move_group(
        std::shared_ptr<
            moveit::planning_interface::MoveGroupInterface
        > move_group)
    {
        move_group_ = move_group;
    }

private:
    void publish_status(const std::string &status)
    {
        std_msgs::msg::String msg;
        msg.data = status;
        status_publisher_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "STATUS = %s",
            status.c_str()
        );
    }

    void home_callback(
        const std_msgs::msg::String::SharedPtr msg)
    {
        if (!move_group_)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "HOME FAILED - MoveGroupInterface is not ready."
            );
            return;
        }

        publish_status("BUSY");

        auto home_target = get_home_target(msg->data);

        if (home_target.empty())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "HOME FAILED - Unknown home location: %s",
                msg->data.c_str()
            );
            publish_status("READY");
            return;
        }
        
        std_msgs::msg::String electro_off_command;
        electro_off_command.data = "ELECTRO OFF";
        
        gripper_command_publisher_->publish(electro_off_command);
        
        RCLCPP_INFO(
            this->get_logger(),
            "ELECTROADHESION OFF: D7 LOW - HOME SAFETY."
        );

        move_group_->clearPathConstraints();
        move_group_->setStartStateToCurrentState();
        move_group_->setJointValueTarget(home_target);

        RCLCPP_INFO(
            this->get_logger(),
            "========== HOME COMMAND RECEIVED =========="
        );

        RCLCPP_INFO(
            this->get_logger(),
            "HOME LOCATION = %s",
            msg->data.c_str()
        );

        moveit::planning_interface::MoveGroupInterface::Plan home_plan;

        auto home_result = move_group_->plan(home_plan);

        if (home_result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "HOME PLAN FAILED."
            );
            publish_status("READY");
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "HOME PLAN SUCCESS - EXECUTING."
        );

        auto home_execute_result =
            move_group_->execute(home_plan);

        if (home_execute_result ==
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "HOME EXECUTION SUCCESS - %s.",
                msg->data.c_str()
            );
            publish_status("READY");
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "HOME EXECUTION FAILED."
            );
            publish_status("READY");
        }
    }

    void mission_callback(
        const tm5_book_msgs::msg::Mission::SharedPtr msg)
    {
        source_location_ = msg->source;
        destination_location_ = msg->destination;

        RCLCPP_INFO(
            this->get_logger(),
            "========== MISSION RECEIVED =========="
        );

        RCLCPP_INFO(
            this->get_logger(),
            "SOURCE      = %s",
            source_location_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "DESTINATION = %s",
            destination_location_.c_str()
        );
        
        auto source_home = get_home_target(source_location_);

        if (!source_home.empty())
        {
            RCLCPP_INFO(
                this->get_logger(),
                "SOURCE HOME SELECTED: %s",
                source_location_.c_str()
            );

            RCLCPP_INFO(
                this->get_logger(),
                "HOME JOINTS: %.5f %.5f %.5f %.5f %.5f %.5f",
                source_home[0],
                source_home[1],
                source_home[2],
                source_home[3],
                source_home[4],
                source_home[5]
            );
        }
    }
      
    std::vector<double> get_home_target(
        const std::string& location)
    {
        if (location == "bottom")
        {
            return {
                -0.00005599,
                 0.69210843,
                 2.42650381,
                -3.13254270,
                 1.57113908,
                 0.00004000
            };
        }

        if (location == "top")
        {
            return {
                -0.00005599,
                -0.25513466,
                 2.08663958,
                -1.84544959,
                 1.57014998,
                 0.00003002
            };
        }

        RCLCPP_ERROR(
            this->get_logger(),
            "Unknown shelf location: %s",
            location.c_str()
        );

        return {};
    }
    
    double get_shelf_z(const std::string& location)
    {
	if (location == "bottom")
    	{
            return 0.195;
    	}

    	if (location == "top")
    	{
       	    return 0.490;
    	}

    	RCLCPP_ERROR(
            this->get_logger(),
            "Unknown shelf location for Z: %s",
            location.c_str()
    	);

    	return -1.0;
    }
    void target_callback(
        const tm5_book_msgs::msg::GraspTarget::SharedPtr msg)
    {
    	RCLCPP_INFO(
     	    this->get_logger(),
     	    "========== CALLBACK FIRED =========="
    	);
    	
        if (!move_group_)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "MoveGroupInterface is not ready."
            );
            return;
        }

        publish_status("BUSY");
        
        // Calculate gripper opening from detected book thickness.
        // Add 3 mm clearance based on physical calibration.
        const double gripper_clearance_mm = 3.0;
        const double gripper_opening_mm =
            msg->thickness_mm + gripper_clearance_mm;

        RCLCPP_INFO(
            this->get_logger(),
            "GRIPPER OPENING = %.2f mm "
            "(thickness %.2f mm + %.2f mm clearance)",
            gripper_opening_mm,
            msg->thickness_mm,
            gripper_clearance_mm
        );

        std_msgs::msg::String gripper_command;
        gripper_command.data =
            "OPEN " + std::to_string(gripper_opening_mm);

        gripper_command_publisher_->publish(gripper_command);

        RCLCPP_INFO(
            this->get_logger(),
            "GRIPPER COMMAND SENT: %s",
            gripper_command.data.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Received GraspTarget: "
            "grasp=(%.3f, %.3f, %.3f) m",
            msg->grasp_position.x,
            msg->grasp_position.y,
            msg->grasp_position.z
        );
        
        geometry_msgs::msg::Pose approach_target;
        geometry_msgs::msg::Pose grasp_target;

        approach_target.position.x = msg->approach_position.x;
        approach_target.position.y = msg->approach_position.y;
        approach_target.position.z = msg->approach_position.z;

        grasp_target.position.x = msg->grasp_position.x;
        grasp_target.position.y = msg->grasp_position.y;
        grasp_target.position.z = msg->grasp_position.z;

        // TEMPORARY +10 DEG ORIENTATION TEST ONLY
        const bool orientation_test_only = false;
        const double test_angle_deg = msg->angle_deg;
        const double test_angle_rad =
            test_angle_deg * M_PI / 180.0;

        const double roll  = 90.0 * M_PI / 180.0;
        const double pitch = -test_angle_rad;
        const double yaw   = 90.0 * M_PI / 180.0;

        const double cr = std::cos(roll * 0.5);
        const double sr = std::sin(roll * 0.5);
        const double cp = std::cos(pitch * 0.5);
        const double sp = std::sin(pitch * 0.5);
        const double cy = std::cos(yaw * 0.5);
        const double sy = std::sin(yaw * 0.5);

        approach_target.orientation.x =
            sr * cp * cy - cr * sp * sy;

        approach_target.orientation.y =
            cr * sp * cy + sr * cp * sy;

        approach_target.orientation.z =
            cr * cp * sy - sr * sp * cy;

        approach_target.orientation.w =
            cr * cp * cy + sr * sp * sy;

        grasp_target.orientation = approach_target.orientation;

        RCLCPP_INFO(
            this->get_logger(),
            "TEMP TEST: RPY = (90.00, %.2f, 90.00) deg",
            -test_angle_deg
        );

        RCLCPP_INFO(
            this->get_logger(),
            "TEMP TEST QUATERNION: (%.6f, %.6f, %.6f, %.6f)",
            approach_target.orientation.x,
            approach_target.orientation.y,
            approach_target.orientation.z,
            approach_target.orientation.w
        );

        move_group_->setStartStateToCurrentState();

        moveit_msgs::msg::Constraints constraints;

        moveit_msgs::msg::JointConstraint j1;
        j1.joint_name = "joint_1";
        j1.position = -0.0003;
        j1.tolerance_above = 0.35;
        j1.tolerance_below = 0.35;
        j1.weight = 1.0;
        constraints.joint_constraints.push_back(j1);

        moveit_msgs::msg::JointConstraint j2;
        j2.joint_name = "joint_2";
        j2.position = 0.50;
        j2.tolerance_above = 1.06;
        j2.tolerance_below = 1.06;
        j2.weight = 1.0;
        constraints.joint_constraints.push_back(j2);

        moveit_msgs::msg::JointConstraint j5;
        j5.joint_name = "joint_5";
        j5.position = 1.5710;
        j5.tolerance_above = 0.1745;
        j5.tolerance_below = 0.1745;
        j5.weight = 1.0;
        constraints.joint_constraints.push_back(j5);

        move_group_->setPathConstraints(constraints);
        move_group_->setPoseTarget(approach_target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        auto result = move_group_->plan(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "APPROACH PLAN SUCCESS - EXECUTING."
            );

            if (!plan.trajectory_.joint_trajectory.points.empty())
            {
                const auto &final_point =
                    plan.trajectory_.joint_trajectory.points.back();

                RCLCPP_INFO(
                    this->get_logger(),
                    "Approach planned final joints (rad):"
                );

                for (size_t i = 0;
                     i < final_point.positions.size();
                     ++i)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "  J%zu = %.6f rad",
                        i + 1,
                        final_point.positions[i]
                    );
                }
            }

            if (orientation_test_only)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "========== ORIENTATION TEST: PLAN ONLY =========="
                );

                move_group_->clearPathConstraints();
                publish_status("READY");
                return;
            }

            auto execute_result = move_group_->execute(plan);

            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "APPROACH EXECUTION SUCCESS."
                );

                // ========================================================
                // APPROACH -> GRASP: CARTESIAN LIN
                //
                // Approach and grasp use the same Y/Z and orientation.
                // Only X changes.
                //
                // Therefore the TCP should move in a straight Cartesian
                // line along X with Y and Z held constant.
                // ========================================================

                move_group_->setStartStateToCurrentState();

                geometry_msgs::msg::Pose grasp_start =
                    approach_target;

                geometry_msgs::msg::Pose grasp_target_lin =
                    grasp_start;

                grasp_target_lin.position.x =
                    grasp_target.position.x;

                RCLCPP_INFO(
                    this->get_logger(),
                    "GRASP LIN START: X=%.3f Y=%.3f Z=%.3f",
                    grasp_start.position.x,
                    grasp_start.position.y,
                    grasp_start.position.z
                );

                RCLCPP_INFO(
                    this->get_logger(),
                    "GRASP LIN TARGET: X=%.3f Y=%.3f Z=%.3f",
                    grasp_target_lin.position.x,
                    grasp_target_lin.position.y,
                    grasp_target_lin.position.z
                );

                std::vector<geometry_msgs::msg::Pose>
                    grasp_waypoints;

                grasp_waypoints.push_back(grasp_start);
                grasp_waypoints.push_back(grasp_target_lin);

                moveit_msgs::msg::RobotTrajectory
                    grasp_trajectory;

                double grasp_fraction =
                    move_group_->computeCartesianPath(
                        grasp_waypoints,
                        0.005,
                        0.0,
                        grasp_trajectory
                    );

                RCLCPP_INFO(
                    this->get_logger(),
                    "GRASP CARTESIAN PATH: %.1f%%",
                    grasp_fraction * 100.0
                );

                if (grasp_fraction >= 0.999)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "GRASP CARTESIAN PATH SUCCESS - EXECUTING."
                    );

                    auto grasp_execute_result =
                        move_group_->execute(grasp_trajectory);

                    if (grasp_execute_result ==
                        moveit::core::MoveItErrorCode::SUCCESS)
                    {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "GRASP EXECUTION SUCCESS."
                        );
                        
                        // Close gripper around the book without forcing it fully closed.
			// Use book thickness + 1 mm holding clearance.
			const double gripper_holding_clearance_mm = 1.0;
			const double gripper_holding_opening_mm =
			    msg->thickness_mm + gripper_holding_clearance_mm;

			std_msgs::msg::String close_command;
			close_command.data =
			    "OPEN " + std::to_string(gripper_holding_opening_mm);

			gripper_command_publisher_->publish(close_command);

			RCLCPP_INFO(
			    this->get_logger(),
			    "GRIPPER HOLDING OPENING = %.2f mm "
			    "(thickness %.2f mm + %.2f mm clearance)",
			    gripper_holding_opening_mm,
			    msg->thickness_mm,
			    gripper_holding_clearance_mm
			);
			
			std_msgs::msg::String electro_on_command;
			electro_on_command.data = "ELECTRO ON";

			gripper_command_publisher_->publish(electro_on_command);

			RCLCPP_INFO(
			    this->get_logger(),
			    "ELECTROADHESION ON: D7 HIGH"
			);

                        move_group_->setStartStateToCurrentState();

                        const double pull_out_distance_mm =
                            msg->width_mm;

                        RCLCPP_INFO(
                            this->get_logger(),
                            "PULL OUT DISTANCE: width=%.1f mm",
                            pull_out_distance_mm
                        );

                        const double first_pull_out_distance_mm = 50.0;

                        geometry_msgs::msg::Pose first_pull_out_target =
                            grasp_target;

                        first_pull_out_target.position.x -=
                            first_pull_out_distance_mm / 1000.0;

                        geometry_msgs::msg::Pose pull_out_target =
                            first_pull_out_target;

                        pull_out_target.position.x -=
                            pull_out_distance_mm / 1000.0;

                        RCLCPP_INFO(
                            this->get_logger(),
                            "FIRST PULL OUT TARGET: X=%.3f Y=%.3f Z=%.3f",
                            first_pull_out_target.position.x,
                            first_pull_out_target.position.y,
                            first_pull_out_target.position.z
                        );

                        RCLCPP_INFO(
                            this->get_logger(),
                            "FINAL PULL OUT TARGET: X=%.3f Y=%.3f Z=%.3f",
                            pull_out_target.position.x,
                            pull_out_target.position.y,
                            pull_out_target.position.z
                        );
			const double source_shelf_z =
                            get_shelf_z(source_location_);
                        const double destination_shelf_z =
                            get_shelf_z(destination_location_);

                        if (source_shelf_z < 0.0 || destination_shelf_z < 0.0)
                        {
                            RCLCPP_ERROR(
                                this->get_logger(),
                                "TRANSFER FAILED - INVALID SOURCE OR DESTINATION SHELF."
                            );
                            publish_status("READY");
                            return;
                        }
                        
                        // Preserve the actual book-centre height relative to the source shelf,
			// then add 5 mm clearance above the destination shelf.
			const double grasp_offset_z =
			    grasp_target.position.z - source_shelf_z;

			const double destination_z =
			    destination_shelf_z +
			    grasp_offset_z +
			    0.005;

			RCLCPP_INFO(
			    this->get_logger(),
			    "GRASP Z OFFSET: %.3f m",
			    grasp_offset_z
			);

			RCLCPP_INFO(
			    this->get_logger(),
			    "DESTINATION Z WITH 5 MM CLEARANCE: %.3f m",
			    destination_z
			);

                        geometry_msgs::msg::Pose transfer_target =
                            pull_out_target;

                        transfer_target.position.z = destination_z;

                        RCLCPP_INFO(
                            this->get_logger(),
                            "TRANSFER TARGET: X=%.3f Y=%.3f Z=%.3f",
                            transfer_target.position.x,
                            transfer_target.position.y,
                            transfer_target.position.z
                        );

                        std::vector<double> home_target = {
                     0.0000,
                     0.6921, // 0.4712
                     2.4265, // 2.2944
                    -3.1325, // -2.7925
                     1.5710,
                     0.0000
                };

                // ========================================================
                // GRASP -> FIRST PULL OUT: CARTESIAN LIN -X
                //
                // Grasp and first pull-out use the same Y/Z and orientation.
                // Only X changes by 50 mm.
                //
                // Therefore the TCP should move in a straight Cartesian
                // line backward along X.
                // ========================================================

                move_group_->setStartStateToCurrentState();

                geometry_msgs::msg::Pose pull_out_start =
                    grasp_target;

                geometry_msgs::msg::Pose first_pull_out_target_lin =
                    pull_out_start;

                first_pull_out_target_lin.position.x =
                    first_pull_out_target.position.x;

                RCLCPP_INFO(
                    this->get_logger(),
                    "PULL 1 LIN START: X=%.3f Y=%.3f Z=%.3f",
                    pull_out_start.position.x,
                    pull_out_start.position.y,
                    pull_out_start.position.z
                );

                RCLCPP_INFO(
                    this->get_logger(),
                    "PULL 1 LIN TARGET: X=%.3f Y=%.3f Z=%.3f",
                    first_pull_out_target_lin.position.x,
                    first_pull_out_target_lin.position.y,
                    first_pull_out_target_lin.position.z
                );

                std::vector<geometry_msgs::msg::Pose>
                    pull_out_waypoints;

                pull_out_waypoints.push_back(pull_out_start);
                pull_out_waypoints.push_back(first_pull_out_target_lin);

                moveit_msgs::msg::RobotTrajectory
                    pull_out_trajectory;

                double pull_out_fraction =
                    move_group_->computeCartesianPath(
                        pull_out_waypoints,
                        0.005,
                        0.0,
                        pull_out_trajectory
                    );

                RCLCPP_INFO(
                    this->get_logger(),
                    "PULL 1 CARTESIAN PATH: %.1f%%",
                    pull_out_fraction * 100.0
                );

                if (pull_out_fraction >= 0.999)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "PULL 1 CARTESIAN PATH SUCCESS - EXECUTING."
                    );

                    auto pull_out_execute_result =
                        move_group_->execute(pull_out_trajectory);

                    if (pull_out_execute_result ==
                        moveit::core::MoveItErrorCode::SUCCESS)
                    {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "PULL OUT EXECUTION SUCCESS."
                        );

                        RCLCPP_INFO(
                            this->get_logger(),
                            "GRIPPER SYNC PLACEHOLDER - READY FOR GRIPPER ACTION."
                        );

                        const double remaining_pull_out_distance_mm =
                            pull_out_distance_mm -
                            first_pull_out_distance_mm;

                        RCLCPP_INFO(
                            this->get_logger(),
                            "REMAINING PULL OUT DISTANCE: %.1f mm",
                            remaining_pull_out_distance_mm
                        );

                        // ========================================================
                        // PULL 1 -> FINAL PULL OUT: CARTESIAN LIN -X
                        //
                        // Pull 1 and final pull-out use the same Y/Z and
                        // orientation. Only X changes.
                        //
                        // Therefore the TCP should move in a straight
                        // Cartesian line backward along X.
                        // ========================================================

                        move_group_->setStartStateToCurrentState();

                        geometry_msgs::msg::Pose second_pull_out_start =
                            first_pull_out_target;

                        geometry_msgs::msg::Pose second_pull_out_target_lin =
                            second_pull_out_start;

                        second_pull_out_target_lin.position.x =
                            pull_out_target.position.x;

                        RCLCPP_INFO(
                            this->get_logger(),
                            "PULL 2 LIN START: X=%.3f Y=%.3f Z=%.3f",
                            second_pull_out_start.position.x,
                            second_pull_out_start.position.y,
                            second_pull_out_start.position.z
                        );

                        RCLCPP_INFO(
                            this->get_logger(),
                            "PULL 2 LIN TARGET: X=%.3f Y=%.3f Z=%.3f",
                            second_pull_out_target_lin.position.x,
                            second_pull_out_target_lin.position.y,
                            second_pull_out_target_lin.position.z
                        );

                        std::vector<geometry_msgs::msg::Pose>
                            second_pull_out_waypoints;

                        second_pull_out_waypoints.push_back(
                            second_pull_out_start
                        );

                        second_pull_out_waypoints.push_back(
                            second_pull_out_target_lin
                        );

                        moveit_msgs::msg::RobotTrajectory
                            second_pull_out_trajectory;

                        double second_pull_out_fraction =
                            move_group_->computeCartesianPath(
                                second_pull_out_waypoints,
                                0.005,
                                0.0,
                                second_pull_out_trajectory
                            );

                        RCLCPP_INFO(
                            this->get_logger(),
                            "PULL 2 CARTESIAN PATH: %.1f%%",
                            second_pull_out_fraction * 100.0
                        );

                        if (second_pull_out_fraction >= 0.999)
                        {
                            RCLCPP_INFO(
                                this->get_logger(),
                                "PULL 2 CARTESIAN PATH SUCCESS - EXECUTING."
                            );

                            auto second_pull_out_execute_result =
                                move_group_->execute(
                                    second_pull_out_trajectory
                                );

                            if (second_pull_out_execute_result ==
                                moveit::core::MoveItErrorCode::SUCCESS)
                            {
                                RCLCPP_INFO(
                                    this->get_logger(),
                                    "SECOND PULL OUT EXECUTION SUCCESS."
                                );

                                // ============================================================
                                // PHASE D-1: J6-ONLY REALIGNMENT
                                //
                                // Use the successful second pull-out endpoint.
                                // J1-J5 are locked near that endpoint.
                                // ONLY J6 is commanded to 0 rad.
                                // ============================================================

                                const auto &second_pull_out_points =
                                    second_pull_out_trajectory.joint_trajectory.points;

                                if (second_pull_out_points.empty())
                                {
                                    RCLCPP_ERROR(
                                        this->get_logger(),
                                        "REALIGN FAILED - SECOND PULL OUT TRAJECTORY IS EMPTY."
                                    );
                                    publish_status("READY");
                                    return;
                                }

                                const auto &second_pull_out_final =
                                    second_pull_out_points.back();

                                if (second_pull_out_final.positions.size() < 6)
                                {
                                    RCLCPP_ERROR(
                                        this->get_logger(),
                                        "REALIGN FAILED - SECOND PULL OUT TRAJECTORY HAS LESS THAN 6 JOINT VALUES."
                                    );
                                    publish_status("READY");
                                    return;
                                }

                                // ============================================================
                                // PHASE D-1: J6-ONLY REALIGNMENT
                                //
                                // The successful second pull-out trajectory already contains
                                // the exact six-joint endpoint reached by the robot.
                                //
                                // J1-J5 MUST remain at those exact values.
                                // ONLY J6 changes to 0 rad.
                                //
                                // IMPORTANT:
                                // Do NOT query /joint_states here.
                                // Do NOT call getCurrentState().
                                // Do NOT call setStartStateToCurrentState().
                                // Do NOT use pose planning / IK / OMPL.
                                // ============================================================

                                const std::vector<double> &pull_out_joints =
                                    second_pull_out_final.positions;

                                if (pull_out_joints.size() < 6)
                                {
                                    RCLCPP_ERROR(
                                        this->get_logger(),
                                        "REALIGN FAILED - SECOND PULL OUT ENDPOINT HAS LESS THAN 6 JOINT VALUES."
                                    );
                                    publish_status("READY");
                                    return;
                                }

                                RCLCPP_INFO(
                                    this->get_logger(),
                                    "REALIGN START - SECOND PULL OUT ENDPOINT:"
                                );

                                for (size_t i = 0; i < 6; ++i)
                                {
                                    RCLCPP_INFO(
                                        this->get_logger(),
                                        "  J%zu = %.6f rad",
                                        i + 1,
                                        pull_out_joints[i]
                                    );
                                }

                                trajectory_msgs::msg::JointTrajectoryPoint
                                    realign_start;

                                trajectory_msgs::msg::JointTrajectoryPoint
                                    realign_end;

                                // ============================================================
                                // D-1 START
                                //
                                // EXACT second pull-out endpoint.
                                //
                                // J1-J5 MUST remain unchanged.
                                // ONLY J6 moves to 0 rad.
                                //
                                // Explicitly provide zero velocities and accelerations
                                // for all six joints.
                                // ============================================================

                                realign_start.positions = pull_out_joints;
                                realign_start.velocities =
                                    std::vector<double>(6, 0.0);
                                realign_start.accelerations =
                                    std::vector<double>(6, 0.0);

                                // End = EXACTLY the same J1-J5.
                                // ONLY J6 changes to zero.
                                realign_end.positions = pull_out_joints;
                                realign_end.positions[5] = 0.0;

                                realign_end.velocities =
                                    std::vector<double>(6, 0.0);
                                realign_end.accelerations =
                                    std::vector<double>(6, 0.0);

                                realign_start.time_from_start =
                                    rclcpp::Duration::from_seconds(0.0);

                                realign_end.time_from_start =
                                    rclcpp::Duration::from_seconds(1.0);

                                moveit_msgs::msg::RobotTrajectory
                                    realign_trajectory;

                                realign_trajectory.joint_trajectory.joint_names =
                                    second_pull_out_trajectory
                                        .joint_trajectory
                                        .joint_names;

                                if (realign_trajectory.joint_trajectory.joint_names.size() != 6)
                                {
                                    RCLCPP_ERROR(
                                        this->get_logger(),
                                        "REALIGN FAILED - EXPECTED 6 JOINT NAMES."
                                    );
                                    publish_status("READY");
                                    return;
                                }

                                realign_trajectory.joint_trajectory.points.push_back(
                                    realign_start
                                );

                                realign_trajectory.joint_trajectory.points.push_back(
                                    realign_end
                                );

                                moveit::planning_interface::MoveGroupInterface::Plan
                                    realign_plan;

                                realign_plan.trajectory_ =
                                    realign_trajectory;

                                RCLCPP_INFO(
                                    this->get_logger(),
                                    "REALIGN: J1-J5 HOLD EXACTLY AT SECOND PULL OUT END."
                                );

                                RCLCPP_INFO(
                                    this->get_logger(),
                                    "REALIGN: J6 %.6f rad -> 0.000000 rad.",
                                    pull_out_joints[5]
                                );

                                RCLCPP_INFO(
                                    this->get_logger(),
                                    "REALIGN TRAJECTORY READY - ONLY J6 MOVES."
                                );

                                auto realign_execute_result =
                                    move_group_->execute(realign_plan);

                                if (realign_execute_result ==
                                    moveit::core::MoveItErrorCode::SUCCESS)
                                {
                                    RCLCPP_INFO(
                                        this->get_logger(),
                                        "REALIGN EXECUTION SUCCESS - J1-J5 UNCHANGED, J6 NOW AT 0."
                                    );

                                        // PHASE D-2: LIFT
                                        //
                                        // Only perform the lift when transferring
                                        // between different shelf rows.
                                        //
                                        // Same-row missions:
                                        //     BOTTOM -> BOTTOM
                                        //     TOP    -> TOP
                                        //
                                        // skip the lift and proceed directly to HOME.
                                        //
                                        // D-1 already established the exact starting joint state:
                                        // J1-J5 = second pull-out endpoint
                                        // J6    = 0 rad
                                        //
                                        // Do NOT query /joint_states.
                                        // Do NOT call getCurrentState().
                                        //
                                        // Use the known D-1 endpoint as the IK seed and solve
                                        // transfer_target from that state.
                                        if (source_location_ != destination_location_)
                                        {
                                            // ========================================================
                                            // PHASE D-2: CARTESIAN LIN Z TRANSFER
                                            //
                                            // Use the exact D-1 joint endpoint as the starting state.
                                            // Do NOT query /joint_states or getCurrentPose().
                                            //
                                            // Only Z changes.
                                            // X, Y and orientation remain unchanged.
                                            //
                                            // BOTTOM -> TOP   : LIN +Z
                                            // TOP    -> BOTTOM: LIN -Z
                                            // ========================================================

                                            const std::vector<double> lift_start_joints =
                                                realign_end.positions;

                                            auto robot_model =
                                                move_group_->getRobotModel();

                                            const moveit::core::JointModelGroup *joint_model_group =
                                                robot_model->getJointModelGroup("tmr_arm");

                                            if (!joint_model_group)
                                            {
                                                RCLCPP_ERROR(
                                                    this->get_logger(),
                                                    "TRANSFER LIN FAILED - COULD NOT GET JOINT MODEL GROUP."
                                                );
                                                publish_status("READY");
                                                return;
                                            }

                                            moveit::core::RobotState lift_start_state(
                                                robot_model
                                            );

                                            lift_start_state.setJointGroupPositions(
                                                joint_model_group,
                                                lift_start_joints
                                            );

                                            lift_start_state.update();

                                            const Eigen::Isometry3d lift_start_transform =
                                                lift_start_state.getGlobalLinkTransform(
                                                    "flange"
                                                );

                                            geometry_msgs::msg::Pose transfer_start;

                                            transfer_start.position.x =
                                                lift_start_transform.translation().x();

                                            transfer_start.position.y =
                                                lift_start_transform.translation().y();

                                            transfer_start.position.z =
                                                lift_start_transform.translation().z();

                                            Eigen::Quaterniond lift_start_quaternion(
                                                lift_start_transform.rotation()
                                            );

                                            transfer_start.orientation.x =
                                                lift_start_quaternion.x();

                                            transfer_start.orientation.y =
                                                lift_start_quaternion.y();

                                            transfer_start.orientation.z =
                                                lift_start_quaternion.z();

                                            transfer_start.orientation.w =
                                                lift_start_quaternion.w();

                                            geometry_msgs::msg::Pose transfer_target_lin =
                                                transfer_start;

                                            transfer_target_lin.position.z =
                                                transfer_target.position.z;

                                            RCLCPP_INFO(
                                                this->get_logger(),
                                                "TRANSFER LIN START: X=%.3f Y=%.3f Z=%.3f",
                                                transfer_start.position.x,
                                                transfer_start.position.y,
                                                transfer_start.position.z
                                            );

                                            RCLCPP_INFO(
                                                this->get_logger(),
                                                "TRANSFER LIN TARGET: X=%.3f Y=%.3f Z=%.3f",
                                                transfer_target_lin.position.x,
                                                transfer_target_lin.position.y,
                                                transfer_target_lin.position.z
                                            );

                                            std::vector<geometry_msgs::msg::Pose>
                                                transfer_waypoints;

                                            transfer_waypoints.push_back(
                                                transfer_start
                                            );

                                            transfer_waypoints.push_back(
                                                transfer_target_lin
                                            );

                                            moveit_msgs::msg::RobotTrajectory
                                                transfer_trajectory;

                                            double transfer_fraction =
                                                move_group_->computeCartesianPath(
                                                    transfer_waypoints,
                                                    0.005,
                                                    0.0,
                                                    transfer_trajectory
                                                );

                                            RCLCPP_INFO(
                                                this->get_logger(),
                                                "TRANSFER CARTESIAN PATH: %.1f%%",
                                                transfer_fraction * 100.0
                                            );

                                            if (transfer_fraction >= 0.999)
                                            {
                                                RCLCPP_INFO(
                                                    this->get_logger(),
                                                    "TRANSFER CARTESIAN PATH SUCCESS - EXECUTING."
                                                );

                                                auto transfer_execute_result =
                                                    move_group_->execute(
                                                        transfer_trajectory
                                                    );

                                                if (transfer_execute_result ==
                                                    moveit::core::MoveItErrorCode::SUCCESS)
                                                {
                                                    RCLCPP_INFO(
                                                        this->get_logger(),
                                                        "TRANSFER EXECUTION SUCCESS."
                                                    );

                                                // ========================================================
                                                // PHASE E-3: DESTINATION INSERTION
                                                //
                                                // Robot is ALREADY at the destination shelf row after D-2.
                                                // Only move the TCP forward along +X by the detected
                                                // book width.
                                                //
                                                // Y, Z and orientation remain unchanged.
                                                // ========================================================

                                                move_group_->setStartStateToCurrentState();

                                                geometry_msgs::msg::Pose insert_start =
                                                    transfer_target;

                                                geometry_msgs::msg::Pose insert_target =
                                                    insert_start;
                                                
                                                insert_target.position.x =
                                                    grasp_target.position.x;

                                                //insert_target.position.x +=
                                                    //msg->width_mm / 1000.0;

                                                RCLCPP_INFO(
                                                    this->get_logger(),
                                                    "INSERT START: X=%.3f Y=%.3f Z=%.3f",
                                                    insert_start.position.x,
                                                    insert_start.position.y,
                                                    insert_start.position.z
                                                );

                                                RCLCPP_INFO(
                                                    this->get_logger(),
                                                    "INSERT TARGET: X=%.3f Y=%.3f Z=%.3f",
                                                    insert_target.position.x,
                                                    insert_target.position.y,
                                                    insert_target.position.z
                                                );

                                                std::vector<geometry_msgs::msg::Pose>
                                                    insert_waypoints;

                                                insert_waypoints.push_back(insert_start);
                                                insert_waypoints.push_back(insert_target);

                                                moveit_msgs::msg::RobotTrajectory
                                                    insert_trajectory;

                                                double insert_fraction =
                                                    move_group_->computeCartesianPath(
                                                        insert_waypoints,
                                                        0.005,
                                                        0.0,
                                                        insert_trajectory
                                                    );

                                                RCLCPP_INFO(
                                                    this->get_logger(),
                                                    "INSERT CARTESIAN PATH: %.1f%%",
                                                    insert_fraction * 100.0
                                                );

                                                if (insert_fraction >= 0.999)
                                                {
                                                    RCLCPP_INFO(
                                                        this->get_logger(),
                                                        "INSERT CARTESIAN PATH SUCCESS - EXECUTING."
                                                    );

                                                    auto insert_execute_result =
                                                        move_group_->execute(insert_trajectory);

                                                    if (insert_execute_result ==
                                                        moveit::core::MoveItErrorCode::SUCCESS)
                                                    {
                                                        RCLCPP_INFO(
                                                            this->get_logger(),
                                                            "INSERT EXECUTION SUCCESS."
                                                        );
                                                        // ========================================================
                                                        // PHASE E-4: GRIPPER RELEASE
                                                        //
                                                        // Turn off electroadhesion first so the book is released.
                                                        // ========================================================
							std_msgs::msg::String electro_off_command;
							electro_off_command.data = "ELECTRO OFF";
							
							gripper_command_publisher_->publish(electro_off_command);
							
                                                        RCLCPP_INFO(
                                                            this->get_logger(),
                                                            "ELECTROADHESION OFF: D7 LOW - BOOK RELEASED."
                                                        );
                                                        
                                                        std_msgs::msg::String release_command;
                                                        release_command.data =
                                                            "OPEN " + std::to_string(gripper_opening_mm);
                                                        gripper_command_publisher_->publish(release_command);
                                                        RCLCPP_INFO(
                                                            this->get_logger(),
                                                                "GRIPPER RELEASE OPENING = %.2f mm",
                                                                    gripper_opening_mm
                                                        );

                                                        // ========================================================
                                                        // PHASE E-5: DESTINATION RETRACT
                                                        //
                                                        // After releasing the book, retract the gripper away from
                                                        // the destination shelf before returning to HOME.
                                                        // Keep destination Z unchanged; only X moves backward.
                                                        // ========================================================

                                                        move_group_->setStartStateToCurrentState();

                                                        geometry_msgs::msg::Pose retract_start =
                                                            insert_target;

                                                        geometry_msgs::msg::Pose retract_target =
                                                            retract_start;

                                                        retract_target.position.x =
                                                            pull_out_target.position.x;

                                                        RCLCPP_INFO(
                                                            this->get_logger(),
                                                            "RETRACT START: X=%.3f Y=%.3f Z=%.3f",
                                                            retract_start.position.x,
                                                            retract_start.position.y,
                                                            retract_start.position.z
                                                        );

                                                        RCLCPP_INFO(
                                                            this->get_logger(),
                                                            "RETRACT TARGET: X=%.3f Y=%.3f Z=%.3f",
                                                            retract_target.position.x,
                                                            retract_target.position.y,
                                                            retract_target.position.z
                                                        );

                                                        std::vector<geometry_msgs::msg::Pose>
                                                            retract_waypoints;

                                                        retract_waypoints.push_back(retract_start);
                                                        retract_waypoints.push_back(retract_target);

                                                        moveit_msgs::msg::RobotTrajectory
                                                            retract_trajectory;

                                                        double retract_fraction =
                                                            move_group_->computeCartesianPath(
                                                                retract_waypoints,
                                                                0.005,
                                                                0.0,
                                                                retract_trajectory
                                                            );

                                                        RCLCPP_INFO(
                                                            this->get_logger(),
                                                            "RETRACT CARTESIAN PATH: %.1f%%",
                                                            retract_fraction * 100.0
                                                        );

                                                        if (retract_fraction >= 0.999)
                                                        {
                                                            RCLCPP_INFO(
                                                                this->get_logger(),
                                                                "RETRACT CARTESIAN PATH SUCCESS - EXECUTING."
                                                            );

                                                            auto retract_execute_result =
                                                                move_group_->execute(retract_trajectory);

                                                            if (retract_execute_result ==
                                                                moveit::core::MoveItErrorCode::SUCCESS)
                                                            {
                                                                RCLCPP_INFO(
                                                                    this->get_logger(),
                                                                    "RETRACT EXECUTION SUCCESS."
                                                                );
                                                            }
                                                            else
                                                            {
                                                                RCLCPP_ERROR(
                                                                    this->get_logger(),
                                                                    "RETRACT EXECUTION FAILED - HOME NOT ATTEMPTED."
                                                                );
                                                                publish_status("READY");
                                                                return;
                                                            }
                                                        }
                                                        else
                                                        {
                                                            RCLCPP_ERROR(
                                                                this->get_logger(),
                                                                "RETRACT CARTESIAN PATH FAILED - HOME NOT ATTEMPTED."
                                                            );
                                                            publish_status("READY");
                                                            return;
                                                        }
                                                    }
                                                    else
                                                    {
                                                        RCLCPP_ERROR(
                                                            this->get_logger(),
                                                            "INSERT EXECUTION FAILED - HOME NOT ATTEMPTED."
                                                        );
                                                        publish_status("READY");
                                                        return;
                                                    }
                                                }
                                                else
                                                {
                                                    RCLCPP_ERROR(
                                                        this->get_logger(),
                                                        "INSERT PLAN FAILED - HOME NOT ATTEMPTED."
                                                    );
                                                    publish_status("READY");
                                                    return;
                                                }
                                            }
                                            else
                                            {
                                                RCLCPP_ERROR(
                                                    this->get_logger(),
                                                    "LIFT EXECUTION FAILED - HOME NOT ATTEMPTED."
                                                );
                                                publish_status("READY");
                                                return;
                                            }
                                        }
                                        else
                                        {
                                            RCLCPP_ERROR(
                                                this->get_logger(),
                                                "LIFT PLAN FAILED - HOME NOT ATTEMPTED."
                                            );
                                            publish_status("READY");
                                            return;
                                        }
                                    }
                                }
                                else
                                {
                                    RCLCPP_ERROR(
                                        this->get_logger(),
                                        "REALIGN EXECUTION FAILED - STOPPING."
                                    );
                                    publish_status("READY");
                                    return;
                                }
                    }
                    else
                    {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "SECOND PULL OUT EXECUTION FAILED - HOME NOT ATTEMPTED."
                        );
                        publish_status("READY");
                        return;
                    }
                }
                else
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "SECOND PULL OUT PLAN FAILED - HOME NOT ATTEMPTED."
                    );
                    publish_status("READY");
                    return;
                }

                // ============================================================
                // HOME
                //
                // Start from the actual post-lift robot state.
                // Explicitly command the ORIGINAL Home joint configuration.
                // ============================================================

                move_group_->clearPathConstraints();
                move_group_->setStartStateToCurrentState();
                
                auto home_target = get_home_target(source_location_);
                
                if (home_target.empty())
		{
                    RCLCPP_ERROR(
        		this->get_logger(),
        		"HOME TARGET INVALID - UNKNOWN SOURCE LOCATION."
                    );
                    publish_status("READY");
                    return;
                }
                
                std_msgs::msg::String home_electro_off_command;
                home_electro_off_command.data = "ELECTRO OFF";
                
                gripper_command_publisher_->publish(home_electro_off_command);
                
                RCLCPP_INFO(
                    this->get_logger(),
                    "ELECTROADHESION OFF: D7 LOW - AUTOMATIC HOME SAFETY."
                );
                
                move_group_->setJointValueTarget(home_target);

                RCLCPP_INFO(
    		    this->get_logger(),
    		    "HOME TARGET: SOURCE = %s",
		    source_location_.c_str()
		);

		RCLCPP_INFO(
   	 	    this->get_logger(),
    		    "HOME JOINTS: %.5f %.5f %.5f %.5f %.5f %.5f",
    		    home_target[0],
    		    home_target[1],
    		    home_target[2],
    		    home_target[3],
    		    home_target[4],
    		    home_target[5]
		);
		
                moveit::planning_interface::MoveGroupInterface::Plan home_plan;

                auto home_result = move_group_->plan(home_plan);

                if (home_result == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "HOME PLAN SUCCESS - EXECUTING."
                    );

                    auto home_execute_result =
                        move_group_->execute(home_plan);

                    if (home_execute_result ==
                        moveit::core::MoveItErrorCode::SUCCESS)
                    {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "HOME EXECUTION SUCCESS."
                        );
                    }
                    else
                    {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "HOME EXECUTION FAILED."
                        );
                    }
                }
                else
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "HOME PLAN FAILED."
                    );
                }
                    }
                    else
                    {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "GRASP EXECUTION FAILED - HOME NOT ATTEMPTED."
                        );
                    }
                }
                else
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "GRASP PLAN FAILED - HOME NOT ATTEMPTED."
                    );
                }
            }
            else
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "APPROACH EXECUTION FAILED - GRASP NOT ATTEMPTED."
                );
            }
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "PLAN FAILED."
            );
        }
    }
    }
        publish_status("READY");
    }

    rclcpp::Subscription<tm5_book_msgs::msg::GraspTarget>::SharedPtr
        subscription_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        gripper_command_publisher_;
        
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        home_subscription_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        status_publisher_;

    rclcpp::Subscription<tm5_book_msgs::msg::Mission>::SharedPtr
        mission_subscription_;

    std::string source_location_ = "bottom";
    std::string destination_location_ = "bottom";

    std::shared_ptr<
        moveit::planning_interface::MoveGroupInterface
    > move_group_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TM5MoveItBridge>();

    auto move_group =
        std::make_shared<
            moveit::planning_interface::MoveGroupInterface
        >(
            node,
            "tmr_arm"
        );

    move_group->setPlanningTime(10.0);
    move_group->setNumPlanningAttempts(10);
    move_group->setMaxVelocityScalingFactor(0.10);
    move_group->setMaxAccelerationScalingFactor(0.10);

    node->set_move_group(move_group);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
