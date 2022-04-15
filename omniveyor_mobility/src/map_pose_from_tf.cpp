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
#include "omniveyor_mobility/map_pose_from_tf.h"

constantTFGaussianEstimator::constantTFGaussianEstimator(std::string& targetFrame, std::string& origin, tf2_ros::Buffer *tfBuffer, int windowLength):
    _tfBuffer(tfBuffer), _targetFrame(targetFrame), _origin(origin), _isExtractedTfFull(false), _windowLen(std::max(2,std::abs(windowLength)))
{
    _extractedTF = Eigen::MatrixXd::Zero(windowLength,7);
    try
    {
        _tf = tfBuffer->lookupTransform(targetFrame, origin, ros::Time(0));
        _lastStamp = _tf.header.stamp;
        Eigen::VectorXd newMeas(7);
        newMeas << _tf.transform.translation.x, _tf.transform.translation.y, _tf.transform.translation.z,
                    _tf.transform.rotation.x, _tf.transform.rotation.y, _tf.transform.rotation.z, _tf.transform.rotation.w;
        _extractedTF.row(0) = newMeas;
        _updateInd = 1;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        // initial guesses
        _tf.header.stamp = ros::Time::now()-ros::Duration(0.1);
        _tf.header.frame_id = origin;
        _tf.child_frame_id = targetFrame;
        _tf.transform.translation.x = _tf.transform.translation.y = _tf.transform.translation.z = 0.;
        _tf.transform.rotation.x = _tf.transform.rotation.y = _tf.transform.rotation.z = 0.;
        _tf.transform.rotation.w = 1.;
        _updateInd = 0;
    }
    _tfCov = Eigen::MatrixXd::Zero(6,6);
    _jacobian = Eigen::MatrixXd::Zero(6,6);
    _updateCov = false; _updateJ = false;
    _tfMean = Eigen::VectorXd(7);
}

bool constantTFGaussianEstimator::update(){
    //std::cout << _targetFrame << "   " << _origin << std::endl;
    if (!_tfBuffer->canTransform(_targetFrame, _origin, ros::Time(0))){
        //std::cout << "cannot transform" << std::endl;
        return false;
    }
    // update transform
    geometry_msgs::TransformStamped newTf = _tfBuffer->lookupTransform(_targetFrame, _origin, ros::Time(0));
    //std::cout << newTf << std::endl;
    if (newTf.header.stamp <= _lastStamp){
        //std::cout << "too old" << std::endl;
        return false;
    }
    // a newer transform has arrived. push to buffer.
    _tf = newTf;
    _lastStamp = _tf.header.stamp;
    Eigen::VectorXd newMeas(7);
    newMeas << _tf.transform.translation.x, _tf.transform.translation.y, _tf.transform.translation.z,
                _tf.transform.rotation.x, _tf.transform.rotation.y, _tf.transform.rotation.z, _tf.transform.rotation.w;
    Eigen::VectorXd oldMeas = _extractedTF.row(_updateInd);
    _extractedTF.row(_updateInd) = newMeas;
    if (++_updateInd >= _windowLen){
        _isExtractedTfFull = true;
        _updateInd = 0;
    }
    if (_isExtractedTfFull)
        _tfMean += (newMeas-oldMeas)/_windowLen;
    else if (_updateInd >= 1)
        _tfMean = _extractedTF.topRows(_updateInd).colwise().mean();
    // set flags
    _updateCov = true;
    _updateJ = true;
    return true;
}

geometry_msgs::TransformStamped constantTFGaussianEstimator::latestTransform(){
    //std::cout << _tf <<std::endl;
    return _tf;
}

Eigen::MatrixXd constantTFGaussianEstimator::latestCov(){
    if (!_updateCov)
        return _tfCov;
    // clear flag
    _updateCov = false;
    // use raw quaternion to prevent singularity
    Eigen::MatrixXd tfCov77(7,7);
    if (_isExtractedTfFull){
        Eigen::MatrixXd centered = _extractedTF.rowwise() - _tfMean.transpose();
        tfCov77 = (centered.adjoint() * centered) / double(_windowLen - 1);
    } else if (_updateInd > 1) {
        Eigen::MatrixXd centered = _extractedTF.topRows(_updateInd).rowwise() - _tfMean.transpose();
        tfCov77 = (centered.adjoint() * centered) / double(_updateInd - 1);
    } else {
        tfCov77 = Eigen::MatrixXd::Zero(7,7);
    }
    // transform covariance
    Eigen::MatrixXd jacobian(6,7);
    Eigen::MatrixXd jacobianQtoRPY(3,4);
    // Jacobian calculation reference: https://www.ucalgary.ca/engo_webdocs/GL/96.20096.JSchleppe.pdf
    // and: https://stats.stackexchange.com/questions/119780/what-does-the-covariance-of-a-quaternion-mean
    // tfMean = [x,y,z,q1,q2,q3,q4]
    double parYawPitchDenom1 = (_tfMean(4)+_tfMean(5))*(_tfMean(4)+_tfMean(5))+(_tfMean(3)+_tfMean(6))*(_tfMean(3)+_tfMean(6));
    double parYawPitchDenom2 = (_tfMean(4)-_tfMean(5))*(_tfMean(4)-_tfMean(5))+(_tfMean(3)-_tfMean(6))*(_tfMean(3)-_tfMean(6));
    double parRollDenom = std::sqrt(1.-4.*(_tfMean(4)*_tfMean(5)+_tfMean(3)*_tfMean(6))*(_tfMean(4)*_tfMean(5)+_tfMean(3)*_tfMean(6)));
    double j21 = -(_tfMean(5)+_tfMean(4))/parYawPitchDenom1-(_tfMean(5)-_tfMean(4))/parYawPitchDenom2;
    double j22 = (_tfMean(6)+_tfMean(3))/parYawPitchDenom1+(_tfMean(6)-_tfMean(3))/parYawPitchDenom2;
    double j23 = (_tfMean(6)+_tfMean(3))/parYawPitchDenom1-(_tfMean(6)-_tfMean(3))/parYawPitchDenom2;
    double j24 = -(_tfMean(5)+_tfMean(4))/parYawPitchDenom1+(_tfMean(5)-_tfMean(4))/parYawPitchDenom2;
    jacobianQtoRPY << 2.*_tfMean(6)/parRollDenom, 2.*_tfMean(5)/parRollDenom, 2.*_tfMean(4)/parRollDenom, 2.*_tfMean(3)/parRollDenom,
                        j21, j22, j23, j24, j24, j23, j22, j21;
    jacobian << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,4), Eigen::MatrixXd::Zero(3,3), jacobianQtoRPY;
    _tfCov = jacobian*tfCov77*jacobian.transpose();
    return _tfCov;
}

Eigen::MatrixXd constantTFGaussianEstimator::covJacobian(){
    if (!_updateJ)
        return _jacobian;
    // clear flag
    _updateJ = false;
    /*
    Jacobian of Translation: [I, 0; Dn, I], where Dn=[0, -z, y; z, 0, -x; -y, x, 0]
    Jacobian of Rotation: [R, 0; 0, R]
    Pose transformation (pre-multiply): _{target} P = T * R * _{origin} P
    Therefore the combined jacobian is JT * JR.
    Reference: https://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
    ***
    The Python implementation of a right multiplication problem:
    def composeHTMCov(covA, aTb, covT):
        Sigma1 = np.array(covA).reshape([6,6])
        rquat = aTb.transform.rotation
        trans = aTb.transform.translation
        R = Rotation.from_quat([rquat.x, rquat.y, rquat.z, rquat.w])
        Sigma2 = np.array(covT).reshape([6,6])
        Dn = np.array([[0., -trans.z, trans.y], [trans.z, 0., -trans.x], [-trans.y, trans.x, 0.]])
        J = np.array([[R, np.zeros([3,3])], [np.dot(Dn, R), R]]).transpose()
        covB = J @ Sigma1 @ J.transpose() + Sigma2
        return covB
    */
    Eigen::Quaterniond q = Eigen::Quaterniond(_tf.transform.rotation.w, _tf.transform.rotation.x,
                                            _tf.transform.rotation.y, _tf.transform.rotation.z);
    _r = q.toRotationMatrix();
    Eigen::MatrixXd Dn(3,3);
    Dn << 0., -_tf.transform.translation.z, _tf.transform.translation.y,
            _tf.transform.translation.z, 0., -_tf.transform.translation.x,
            -_tf.transform.translation.y, _tf.transform.translation.x, 0.;
    _jacobian << _r, Eigen::Matrix3d::Zero(), Dn*_r, _r;
    return _jacobian;
}

Eigen::MatrixXd constantTFGaussianEstimator::transformCov(Eigen::MatrixXd &covIn){
    // recursive call to in-class methods to ensure fetching the latest values.
    Eigen::MatrixXd J = covJacobian();
    // _jacobianT has been updated after the function call.
    return J * covIn * J.transpose() + latestCov();
}

mapPoseFromTFOdom_node::mapPoseFromTFOdom_node(ros::NodeHandle *node): _nh(*node){
    _tfListener = new tf2_ros::TransformListener(_tfBuffer);
    _nh.param<std::string>("map_pose_topic", _poseTopic, "map_pose");
    _nh.param<std::string>("map_frame", _mapFrame, "map");
    _nh.param<std::string>("odom_topic", _odomTopic, "odom");
    _nh.param<std::string>("odom_frame", _odomFrame, "");
    _nh.param<std::string>("base_frame", _baseFrame, "");
    _nh.param<int>("tf_covariance_estimation_window", _windowLen, 50);
    _nh.param<double>("publish_rate", _pubRate, 40.0);
    _nh.param<bool>("publish_tf_cov", _pubTfCov, true);
    // in case these frames are not provided as params
    if (_odomFrame=="" or _baseFrame==""){
        while (_nh.ok()){
            // wait for the first odom message to come in, such that odom and base link frames are obtained.
            nav_msgs::Odometry::ConstPtr odo = ros::topic::waitForMessage<nav_msgs::Odometry>
                                                        (_odomTopic, _nh, ros::Duration(0.1));
            if (odo == nullptr)
                continue;
            _lastOdom = *odo;
            _odomFrame = _lastOdom.header.frame_id;
            _baseFrame = _lastOdom.child_frame_id;
            break;
        }
    }
    ros::Subscriber sub = _nh.subscribe(_odomTopic, 10, &mapPoseFromTFOdom_node::odomSubsCb, this);
    _odomMapGaussianEst = new constantTFGaussianEstimator(_mapFrame, _odomFrame, &_tfBuffer, _windowLen);
    if (_nh.ok())
    {
        try
        {
            _tfOdomBase = _tfBuffer.lookupTransform(_odomFrame, _baseFrame, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            // initial guesses
            _tfOdomBase.header.stamp = ros::Time::now()-ros::Duration(0.1);
            _tfOdomBase.header.frame_id = _odomFrame;
            _tfOdomBase.child_frame_id = _baseFrame;
            _tfOdomBase.transform.translation.x = _tfOdomBase.transform.translation.y = _tfOdomBase.transform.translation.z = 0.;
            _tfOdomBase.transform.rotation.x = _tfOdomBase.transform.rotation.y = _tfOdomBase.transform.rotation.z = 0.;
            _tfOdomBase.transform.rotation.w = 1.0;
        }
    }
    _mapPosePublisher = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(_poseTopic, 2);
    if (_pubTfCov){
        _tfCovPublisher = _nh.advertise<std_msgs::Float64MultiArray>(_poseTopic+"/tf_cov", 2);
        //Clear array
        _tfCov.data.clear();
        //for loop, pushing data in the size of the array
        for (int i = 0; i < 36; i++)
        {
            _tfCov.data.push_back(0.0);
        }
    }
}

mapPoseFromTFOdom_node::~mapPoseFromTFOdom_node(){
    delete _tfListener;
    delete _odomMapGaussianEst;
}

void mapPoseFromTFOdom_node::odomSubsCb(const nav_msgs::Odometry::ConstPtr& msg){
    _lastOdom = *msg;
}

void mapPoseFromTFOdom_node::run(){
    ros::Rate rate(_pubRate);
    while (_nh.ok())
    {
        std_msgs::Header hdr;
        Eigen::Vector3d trans;
        Eigen::Quaterniond rot;
        //std::cout << "in run" << std::endl;
        _odomMapGaussianEst->update();
        if (_tfBuffer.canTransform(_odomFrame, _baseFrame, ros::Time(0))){
            _tfOdomBase = _tfBuffer.lookupTransform(_odomFrame, _baseFrame, ros::Time(0));
        }
        //std::cout << tfOdomBase <<std::endl;
        if (_tfOdomBase.header.stamp < _lastMapPose.header.stamp && _lastOdom.header.stamp < _lastMapPose.header.stamp) {
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        if (_lastOdom.header.stamp < _tfOdomBase.header.stamp){
            // got an updated transform
            hdr = _tfOdomBase.header;
            trans << _tfOdomBase.transform.translation.x, _tfOdomBase.transform.translation.y, _tfOdomBase.transform.translation.z;
            rot = Eigen::Quaterniond(_tfOdomBase.transform.rotation.w, _tfOdomBase.transform.rotation.x,
                                        _tfOdomBase.transform.rotation.y, _tfOdomBase.transform.rotation.z);
        } else {
            // use the odom msg
            hdr = _lastOdom.header;
            trans << _lastOdom.pose.pose.position.x, _lastOdom.pose.pose.position.y, _lastOdom.pose.pose.position.z;
            rot = Eigen::Quaterniond(_lastOdom.pose.pose.orientation.w, _lastOdom.pose.pose.orientation.x,
                                        _lastOdom.pose.pose.orientation.y, _lastOdom.pose.pose.orientation.z);
        }
        // transform pose stamped
        geometry_msgs::PoseStamped odomPose;
        odomPose.header = hdr;
        odomPose.pose.position.x = trans(0); odomPose.pose.position.y = trans(1); odomPose.pose.position.z = trans(2);
        odomPose.pose.orientation.x = rot.x(); odomPose.pose.orientation.y = rot.y();
        odomPose.pose.orientation.z = rot.z(); odomPose.pose.orientation.w = rot.w();
        geometry_msgs::PoseStamped ps;
        tf2::doTransform(odomPose, ps, _odomMapGaussianEst->latestTransform());
        Eigen::MatrixXd odomBaseCov = Eigen::Map<Eigen::MatrixXd>(_lastOdom.pose.covariance.data(), 6, 6);
        Eigen::MatrixXd covTransformed = _odomMapGaussianEst->transformCov(odomBaseCov);
        if (_pubTfCov){
            Eigen::MatrixXd tfCov = _odomMapGaussianEst->latestCov();
            double *covData = tfCov.data();
		    for (int i = 0; i < 36; i++)
			    _tfCov.data[i] = covData[i];
            _tfCovPublisher.publish(_tfCov);
        }
        Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> covTransformedRowMaj(covTransformed);
        double *covData = covTransformedRowMaj.data();
        // assembly
        _lastMapPose.header.frame_id = _mapFrame;
        _lastMapPose.header.stamp = hdr.stamp;
        _lastMapPose.pose.pose = ps.pose;
        for (int i  =0; i < 36; i++)
            _lastMapPose.pose.covariance[i] = covData[i];
        _mapPosePublisher.publish(_lastMapPose);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_pose_from_tf_odom");

    ros::NodeHandle node;
    mapPoseFromTFOdom_node repub(&node);
    //std::cout << "initialized" << std::endl;
    repub.run();
}
