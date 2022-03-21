#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <dynamic_reconfigure/server.h>
#include <nav_core/base_local_planner.h>
#include "omniveyor_mobility/PIDControllerConfig.h"

using namespace std;

namespace pid_controller{

  class PIDController : public nav_core::BaseLocalPlanner{

    public:
      PIDController();

      PIDController(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      ~PIDController();
      
      void reconfigureCb(PIDControllerConfig& config,uint32_t level);

      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);
      
      void acceptPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);

      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);

      bool getTransformedPosition(geometry_msgs::PoseStamped &pose, double *x,double *y, double *th);

      void calculatePIDController(geometry_msgs::Twist& cmd_vel, bool yieldOutput);

      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      bool isGoalReached();

    private:
      std::string base_frame_;
      bool initialized_, goal_reached_, trackingLastGoal_;
      int plan_index_;
      bool overwrite_global_plan_orientation_;
      bool publish_look_ahead_point_;
      double lin_lookahead_, ang_lookahead_, lin_tol_, ang_tol_;
      double max_vel_lin_, max_acc_lin_, max_vel_ang_, max_acc_ang_, max_int_lin_, max_int_ang_;
      double kp_lin_, ki_lin_, kd_lin_, kp_ang_, ki_ang_, kd_ang_;
      double lastTime_;
      double intErrX_, intErrY_, intErrTh_;
      double errX_, errY_, errTh_;
      double vbX_, vbY_, vbTh_;
      double lastErrX_, lastErrY_, lastErrTh_;
      double vxLast_, vyLast_, vthLast_;
      double dt_;

      std::vector<geometry_msgs::PoseStamped> global_plan_;
      geometry_msgs::PoseStamped currentGoal_;
      base_local_planner::OdometryHelperRos *odom_helper_;
      tf2_ros::Buffer *tf_;
      ros::Publisher target_pose_pub_;

      // dynamic reconfigure
      dynamic_reconfigure::Server<PIDControllerConfig> *dsrv_;
      void reconfigureCB(PIDControllerConfig &config, uint32_t level);
  };
};

#endif