#include "omniveyor_mobility/pid_controller.h"

#include <pluginlib/class_list_macros.h>

using namespace std;

PLUGINLIB_EXPORT_CLASS(pid_controller::PIDController, nav_core::BaseLocalPlanner)

namespace pid_controller{

    constexpr double kControllerFrequency = 20.0;

    PIDController::PIDController() : tf_(NULL), initialized_(false) {}

    PIDController::PIDController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
        : tf_(NULL), initialized_(false)
    {
        initialize(name, tf, costmap_ros);
    }

    PIDController::~PIDController() {}

    void PIDController::reconfigureCb(PIDControllerConfig& config,uint32_t level) {
        // settings
        base_frame_ = config.base_frame;
        overwrite_global_plan_orientation_ = config.overwrite_global_plan_orientation;
        publish_look_ahead_point_ = config.publish_look_ahead_point;
        // target
        lin_lookahead_ = fabs(config.linear_lookahead);
        ang_lookahead_ = fabs(config.angular_lookahead);
        // goal tolerance
        lin_tol_ = fabs(config.linear_precision);
        ang_tol_ = fabs(config.angular_precision);
        // linear
        max_vel_lin_ = fabs(config.max_vel_lin);
        max_int_lin_ = fabs(config.max_int_lin);
        max_acc_lin_ = fabs(config.max_acc_lin);
        // angular
        max_vel_ang_ = fabs(config.max_vel_ang);
        max_int_ang_ = fabs(config.max_int_ang);
        max_acc_ang_ = fabs(config.max_acc_ang);
        // pid controller params
        kp_lin_ = config.kp_lin;
        ki_lin_ = config.ki_lin;
        kd_lin_ = config.kd_lin;

        kp_ang_ = config.kp_ang;
        ki_ang_ = config.ki_ang;
        kd_ang_ = config.kd_ang;
    }

    void PIDController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_){
            ros::NodeHandle nh = ros::NodeHandle("~/" + name);
            tf_ = tf;
            string odom_topic;
            string kOdomTopic = "odom";
            nh.param("odom_topic", odom_topic, kOdomTopic);
            odom_helper_ = new base_local_planner::OdometryHelperRos(odom_topic);
            target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 10);
            
            // dynamic reconfigure
            dsrv_ = new dynamic_reconfigure::Server<PIDControllerConfig>(nh);
            dynamic_reconfigure::Server<PIDControllerConfig>::CallbackType cb =
                boost::bind(&PIDController::reconfigureCb, this, _1, _2);
            dsrv_->setCallback(cb);

            //base_frame_ = "/base_link";
            plan_index_ = 0;
            goal_reached_ = false;
            initialized_ = true;

            // time interval
            double controller_freqency;
            nh.param("/move_base/controller_frequency", controller_freqency,
                kControllerFrequency);
            dt_ = 1. / controller_freqency;

            ROS_INFO("PID Local Planner initialized!");
        } else {
            ROS_WARN("PID Local Planner has already been initialized.");
        }
    }

    void PIDController::acceptPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan){
        // set new plan
        global_plan_.clear();
        global_plan_ = global_plan;
        if (overwrite_global_plan_orientation_){
            for (int n = 0; n < global_plan_.size()-1; n++){
                double g_th = atan2((global_plan_[n+1].pose.position.y -
                                global_plan_[n].pose.position.y),
                                (global_plan_[n+1].pose.position.x -
                                global_plan_[n].pose.position.x));
                tf2::Quaternion g_quat;
                g_quat.setEuler(0., 0., g_th);
                global_plan_[n].pose.orientation.x = g_quat[0];
                global_plan_[n].pose.orientation.y = g_quat[1];
                global_plan_[n].pose.orientation.z = g_quat[2];
                global_plan_[n].pose.orientation.w = g_quat[3];
            }
        }
    }

    bool PIDController::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
    {
        if(!initialized_){
            ROS_ERROR("PID Local Planner has not been initialized!");
            return false;
        }
        geometry_msgs::PoseStamped newGoal = global_plan.back();
        if (hypot(newGoal.pose.position.x-currentGoal_.pose.position.x,
                    newGoal.pose.position.y-currentGoal_.pose.position.y)<=lin_tol_){
            double roll, pitch, yaw;
            double rollNew, pitchNew, yawNew;
            tf2::Matrix3x3 r = tf2::Matrix3x3(tf2::Quaternion(currentGoal_.pose.orientation.x, 
                                                                currentGoal_.pose.orientation.y,
                                                                currentGoal_.pose.orientation.z,
                                                                currentGoal_.pose.orientation.w));
            tf2::Matrix3x3 rNew = tf2::Matrix3x3(tf2::Quaternion(newGoal.pose.orientation.x, 
                                                                newGoal.pose.orientation.y,
                                                                newGoal.pose.orientation.z,
                                                                newGoal.pose.orientation.w));
            r.getRPY(roll, pitch, yaw);
            rNew.getRPY(rollNew, pitchNew, yawNew);
            if (fabs(yawNew-yaw)<=ang_tol_){
                if (trackingLastGoal_){
                    // we are already in the Do Not Disturb finalization.
                    // This is literally the same goal. Do Not Disturb.
                    return true;
                }
                else{
                    // set new plan
                    acceptPlan(global_plan);
                    // reset plan parameters only
                    plan_index_ = 0;
                    goal_reached_ = false;
                    return true;
                }
            }
        }
        // set new plan
        acceptPlan(global_plan);
        // The goal has changed. Disable lock.
        trackingLastGoal_ = false;
        // reset plan parameters
        plan_index_ = 0;
        goal_reached_ = false;
        // reset pid
        intErrX_ = intErrY_ = intErrTh_ = 0.0;
        errX_ = errY_ = errTh_ = 0.0;
        vbX_ = vbY_ = vbTh_ = 0.0;
        lastErrX_ = lastErrY_ = lastErrTh_ = 0.0;
        vxLast_ = vyLast_ = vthLast_ = 0.0;
        lastTime_ = ros::Time::now().toSec();
        return true;
    }

    bool PIDController::getTransformedPosition(geometry_msgs::PoseStamped &pose, double *x, double *y, double *th) {
        pose.header.stamp = ros::Time::now();  // last transformation available
        // If not transformable, return infinite.
        if (tf_->canTransform(base_frame_, pose.header.frame_id, ros::Time(0))){
            geometry_msgs::PoseStamped ps;
            geometry_msgs::TransformStamped trans = tf_->lookupTransform(
                            base_frame_, pose.header.frame_id, ros::Time(0));
            tf2::doTransform(pose, ps, trans);
            *x = ps.pose.position.x;
            *y = ps.pose.position.y;
            tf2::Matrix3x3 r = tf2::Matrix3x3(tf2::Quaternion(ps.pose.orientation.x, ps.pose.orientation.y,
                                                ps.pose.orientation.z, ps.pose.orientation.w));
            double pitch, roll;
            r.getRPY(roll, pitch, *th);
            //ROS_WARN("%f, %f, %f", *x, *y, *th);
            return true;
        } else {
            *x = 0.;
            *y = 0.;
            *th = 0.;
            ROS_WARN_STREAM_THROTTLE(2.0, "Transformation failed from" << pose.header.frame_id << "->" << base_frame_);
            return false;
        }
        /*
        geometry_msgs::PoseStamped ps;
        try{
            ps = tf_->transform(pose, base_frame_);
        } catch(tf2::TransformException& ex) {
            ROS_WARN_STREAM_THROTTLE(2.0, "Could not obtain transform from " << pose.header.frame_id <<
                                    " to " << base_frame_ << ". Error was " << ex.what() << "\n");
            *x = 0.;
            *y = 0.;
            *th = 0.;
            return false;
        }
        *x = ps.pose.position.x;
        *y = ps.pose.position.y;
        tf2::Matrix3x3 r = tf2::Matrix3x3(tf2::Quaternion(ps.pose.orientation.x, ps.pose.orientation.y,
                                            ps.pose.orientation.z, ps.pose.orientation.w));
        double pitch, roll;
        r.getRPY(roll, pitch, *th);
        return true;
        */
        //std::cout << base_frame_ << " x: "<< *x << " y: "<< *y << " ,th: " << *theta << std::endl;
    }

    void PIDController::calculatePIDController(geometry_msgs::Twist& cmd_vel, bool yieldOutput){
        double timeNow = ros::Time::now().toSec();
        double dt = fmax(timeNow - lastTime_, dt_);
        if (yieldOutput){
            double vEx, vEy, vEth;
            vEx = (errX_ - lastErrX_)/dt - vbX_;
            vEy = (errY_ - lastErrY_)/dt - vbY_;
            vEth = (errTh_ - lastErrTh_)/dt - vbTh_;
            lastErrX_ = errX_;
            lastErrY_ = errY_;
            lastErrTh_ = errTh_;
            intErrX_ += errX_ * dt;
            intErrY_ += errY_ * dt;
            intErrTh_ += errTh_ * dt;
            intErrX_ = copysign(fmin(max_int_lin_, fabs(intErrX_)), intErrX_);
            intErrY_ = copysign(fmin(max_int_lin_, fabs(intErrY_)), intErrY_);
            intErrTh_ = copysign(fmin(max_int_ang_, fabs(intErrTh_)), intErrTh_);
            double vx = kp_lin_ * errX_ + ki_lin_ * intErrX_ + kd_lin_ * vEx;
            double vy = kp_lin_ * errY_ + ki_lin_ * intErrY_ + kd_lin_ * vEy;
            double vth = kp_ang_ * errTh_ + ki_ang_ * intErrTh_ + kd_ang_ * vEth;
            //ROS_WARN("%f, %f, %f",vx, vy, vth);
            double ax = (vx - vxLast_)/dt;
            double ay = (vy - vyLast_)/dt;
            double ath = (vth - vthLast_)/dt;
            double scaleLinAcc = max_acc_lin_ / (hypot(ax, ay)+1e-9);
            double scaleAngAcc = max_acc_ang_ / (fabs(ath)+1e-9);
            if (scaleLinAcc >= 1.){
                cmd_vel.linear.x = vx;
                cmd_vel.linear.y = vy;
            } else {
                cmd_vel.linear.x = vxLast_ + (vx - vxLast_)*scaleLinAcc;
                cmd_vel.linear.y = vyLast_ + (vy - vyLast_)*scaleLinAcc;
            }
            if (scaleAngAcc >= 1.)
                cmd_vel.angular.z = vth;
            else
                cmd_vel.angular.z = vthLast_ + (vth - vthLast_)*scaleAngAcc;
            double scaleLin = max_vel_lin_ / (hypot(cmd_vel.linear.x, cmd_vel.linear.y)+1e-9);
            double scaleAng = max_vel_ang_ / (fabs(cmd_vel.angular.z)+1e-9);
            if (scaleLin < 1.){
                cmd_vel.linear.x *= scaleLin;
                cmd_vel.linear.y *= scaleLin;
            }
            if (scaleAng < 1.)
                cmd_vel.angular.z *= scaleAng;
            vxLast_ = cmd_vel.linear.x;
            vyLast_ = cmd_vel.linear.y;
            vthLast_ = cmd_vel.angular.z;
        } else {
            cmd_vel.linear.x = 0.;
            cmd_vel.linear.y = 0.;
            cmd_vel.angular.z = 0.;
            vxLast_ = vyLast_ = vthLast_ = 0.;
        }
        lastTime_ = timeNow;
    }

    bool PIDController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if(!initialized_){
            ROS_ERROR("PID Local Planner has not been initialized!");
            return false;
        }
        if (goal_reached_) {
            ROS_ERROR("PID Local Planner goal already reached.");
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            return true;
        }

        bool isValid;
        // Find first target pose beyond the lookahead window
        while (!trackingLastGoal_ && plan_index_ < global_plan_.size()-1) {
            currentGoal_ = global_plan_[plan_index_];
            // if not transformable, do not move.
            isValid = getTransformedPosition(currentGoal_, &errX_, &errY_, &errTh_);
            if (isValid && hypot(errX_, errY_)< lin_lookahead_ && fabs(errTh_)< ang_lookahead_){
                // This pose has reached. Propagate to next goal in the next control cycle.
                plan_index_++;
            } else {
                break;
            }
        }
        if (trackingLastGoal_ || plan_index_ >= global_plan_.size()-1) {
            if (!trackingLastGoal_){
                // locks last goal until convergence, unless a different goal is invoked.
                trackingLastGoal_ = true;
                currentGoal_ = global_plan_.back();
            }
            isValid = getTransformedPosition(currentGoal_, &errX_, &errY_, &errTh_);
            if (isValid){
                double linErr = hypot(errX_, errY_);
                double angErr = fabs(errTh_);
                // finalize last goal reaching
                if (linErr < lin_tol_ && angErr < ang_tol_)
                    goal_reached_ = true;
            }
        }

        // publish target pose
        if (publish_look_ahead_point_){
            geometry_msgs::PoseStamped target = global_plan_[plan_index_];
            target.header.frame_id = "map";
            target.header.stamp = ros::Time::now();
            target_pose_pub_.publish(target);
        }

        // odometry for robot velocities in local frame
        nav_msgs::Odometry base_odom;
        odom_helper_->getOdom(base_odom);
        vbX_ = base_odom.twist.twist.linear.x;
        vbY_ = base_odom.twist.twist.linear.y;
        vbTh_ = base_odom.twist.twist.angular.z;

        // PID control loop
        //geometry_msgs::Twist cmd_vel_1;
        calculatePIDController(cmd_vel, (isValid && !goal_reached_));
        
        return true;
    }

    bool PIDController::isGoalReached()
    {
        if(!initialized_){
            ROS_ERROR("PID Local Planner has not been initialized!");
            return false;
        }
        if (goal_reached_) {
            ROS_ERROR("PID Local Planner goal reached.");
            return true;
        }
        return false;
    }
}
