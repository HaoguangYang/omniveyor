/**
 * @file map_pose_from_tf.h
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief Helper node that republishes the TF(map->base_link) as a PoseWithCovarianceStamped, with properly calculated covariance.
 * @version 0.1
 * @date 2022-03-27
 * 
 * @copyright Copyright (c) 2022 Haoguang Yang
 * 
 */
#ifndef _MAP_POSE_FROM_TF_H_
#define _MAP_POSE_FROM_TF_H_
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/buffer_core.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class constantTFGaussianEstimator{
    public:

        /**
         * @brief Construct a new constant TF Gaussian Estimator object. The object assumes a near-constant transform to be
         * a white noise process, and estimates its covariance.
         * 
         * @param targetFrame Target frame of the transformation, such that $_{target}T^{origin} * _{origin}X = _{target}X.$
         * @param origin Origin frame of the transform.
         * @param tfBuffer Pointer to an initialized tf2_ros::Buffer, with its tf2_ros::TransformListener already attached.
         * @param windowLength How many consecutive TF are collected over time to calculate the covariance.
         */
        constantTFGaussianEstimator(std::string& targetFrame, std::string& origin, tf2_ros::Buffer *tfBuffer, int windowLength);

        /**
         * @brief Destroy the constant TF Gaussian Estimator object
         * 
         */
        virtual ~constantTFGaussianEstimator() = default;

        /**
         * @brief Update the internal states: push the latest TF to the record and update the mean over time.
         * This function has to be called every cycle before other latest* variables are updated.
         * 
         * @return true - A newer TF has been updated to the record.
         * @return false - No newer TF is available. Last values are maintained.
         */
        bool update();

        /**
         * @brief Return the latest transform from origin to target.
         * 
         * @return geometry_msgs::TransformStamped 
         */
        geometry_msgs::TransformStamped latestTransform();

        /**
         * @brief Calculate and return the lastest covariance matrix of the monitored transform.
         * 
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd latestCov();

        /**
         * @brief Calculate and return the Jacobian matrix for transforming the covariance matrix of measurements
         * in origin frame to target frame.
         * 
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd covJacobian();

        /**
         * @brief Helper function that transforms the observation covariances in origin frame to target frame.
         * This function calls latestCov() and covJacobian() internally.
         * 
         * @param covIn Observation covariance of values in origin frame.
         * @return Eigen::MatrixXd Transformed covariance, with the covariance of the transform imposed.
         */
        Eigen::MatrixXd transformCov(Eigen::MatrixXd &covIn);

        /**
         * @brief Override internal covariance estimate with input of transformed covariance from external sources.
         */
        //void externalUpdateTransformedCov(Eigen::MatrixXd &covIn);

    protected:
        std::string _targetFrame, _origin;
        tf2_ros::Buffer *_tfBuffer;
        geometry_msgs::TransformStamped _tf;
        Eigen::MatrixXd _tfCov;
        Eigen::MatrixXd _jacobian;
        Eigen::MatrixXd _r;
        Eigen::MatrixXd _extractedTF;
        Eigen::VectorXd _tfMean;
        bool _isExtractedTfFull;
        unsigned int _windowLen;
        unsigned int _updateInd;
        ros::Time _lastStamp;
        bool _updateCov, _updateJ;
};

class mapPoseFromTFOdom_node{
    public:

        /**
         * @brief Construct a new mapPoseFromTFOdom node object. This object republishes a TF (map->base_link) as a
         * PoseWithCovarianceStamped message under topic specified by param map_pose_topic.
         * 
         * @param node The ROS node handle pointer.
         */
        mapPoseFromTFOdom_node(ros::NodeHandle *node);

        /**
         * @brief Destroy the mapPoseFromTFOdom node object
         * 
         */
        ~mapPoseFromTFOdom_node();

        /**
         * @brief Callback function of the odometry subscriber. It stores the message to internal variable of the class.
         * 
         * @param msg 
         */
        void odomSubsCb(const nav_msgs::Odometry::ConstPtr& msg);

        /**
         * @brief Callback function of the slowly-updating pose topic. It stores the message to internal variable of the class.
         * 
         * @param msg subscribed incoming message.
         */
        void slowPoseSubsCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

        /**
         * @brief Main (blocking) execution of the transform republisher loop.
         * 
         */
        void run();

    protected:
        ros::NodeHandle _nh;
        ros::Publisher _mapPosePublisher, _tfCovPublisher;
        constantTFGaussianEstimator *_odomMapGaussianEst;
        std::string _poseTopic, _mapFrame, _odomTopic, _odomFrame, _baseFrame, _slowPoseTopic;
        double _pubRate;
        int _windowLen;
        bool _pubTfCov;
        bool _fuseSlowPose;
        tf2_ros::Buffer _tfBuffer;
        tf2_ros::TransformListener *_tfListener;
        geometry_msgs::TransformStamped _tfOdomBase;
        nav_msgs::Odometry _lastOdom;
        geometry_msgs::PoseWithCovarianceStamped _lastSlowPose;
        geometry_msgs::PoseWithCovarianceStamped _lastMapPose;
        std_msgs::Float64MultiArray _tfCov;
};

#endif
