#!/usr/bin/python

import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Point, Quaternion, TransformStamped
import tf2_ros
import rospy
from typing import Union

class AbortionRecord:
    """Record embedded in the planner module indicating
    where the module has failed previously.
    """
    def __init__(self, goal, abortedPose, context=None, recordTime=None):
        """Constructor of the record class

        Args:
            goal (Any): Goal that failed to reach
            abortedPose (Any): Pose where it failed
            context (Any, optional): High Level Context when it failed. 
            Defaults to None.
            recordTime (Any, optional): Time when it failed. Defaults to None.
        """
        self.goal = goal
        self.abortedPose = abortedPose
        self.context = context
        self.recordTime = recordTime

def linearDiff(pt1:Point, pt2:Point)->np.array:
    """Returns the vector pointing from Pt2 to Pt1.

    Args:
        pt1 (Point): Point 1
        pt2 (Point): Point 2

    Returns:
        np.array: Point 1 - Point 2
    """
    return np.array([pt1.x, pt1.y, pt1.z]) - np.array([pt2.x, pt2.y, pt2.z])

def angleDiff(ang1:Union[list,np.array], ang2:Union[list,np.array])->np.array:
    """Returns the angular difference between two Euler Angle Vectors.

    Args:
        ang1 (Union[list,np.array]): Euler Angle 1
        ang2 (Union[list,np.array]): Euler Angle 2

    Returns:
        np.array: Euler Angle 1 - Euler Angle 2. Rounded to range [-pi, pi).
    """
    return np.mod((np.array(ang1)-np.array(ang2)+np.pi), np.pi+np.pi)-np.pi

def rpyFromQuaternion(quat:Quaternion)->np.array:
    """Helper function that converts ROS Message Quaternion to Roll Pitch Yaw.

    Args:
        quat (Quaternion): Input quaternion.

    Returns:
        np.array: Roll, Pitch, Yaw angles.
    """
    return Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')

def quaternionDiff(quat1:Quaternion, quat2:Quaternion)->Quaternion:
    r1 = Rotation.from_quat([quat1.x, quat1.y, quat1.z, quat1.w])
    r2 = Rotation.from_quat([-quat2.x, -quat2.y, -quat2.z, quat2.w])
    r = Rotation.concatenate(r1, r2).as_quat()
    return Quaternion(x=r[0], y=r[1], z=r[2], w=r[3])

def ptInRectangle(point:Point, verts:list):
    verts = np.array(verts)
    a1 = abs( (verts[1][0] * verts[0][1] - verts[0][0] * verts[1][1]) +
                (point.x * verts[1][1] - verts[1][0] * point.y) +
                (verts[0][0] * point.y - point.x * verts[0][1]) )
    a2 = abs( (verts[2][0] * verts[1][1] - verts[1][0] * verts[2][1]) +
                (point.x * verts[2][1] - verts[2][0] * point.y) +
                (verts[1][0] * point.y - point.x * verts[1][1]) )
    a3 = abs( (verts[3][0] * verts[2][1] - verts[2][0] * verts[3][1]) +
                (point.x * verts[3][1] - verts[3][0] * point.y) +
                (verts[2][0] * point.y - point.x * verts[2][1]) )
    a4 = abs( (verts[0][0] * verts[3][1] - verts[3][0] * verts[0][1]) +
                (point.x * verts[0][1] - verts[0][0] * point.y) +
                (verts[3][0] * point.y - point.x * verts[3][1]) )
    e1 = verts[1]-verts[0]
    e2 = verts[2]-verts[1]
    # v1.x*v2.y - v1.y*v2.x
    rectA = abs(np.cross(e1, e2))
    return (a1+a2+a3+a4) <= (rectA+rectA)

def composeHTMCov(covB:Union[list,tuple], aTb:TransformStamped,
                    covT:Union[list,tuple]=np.zeros([6,6]))->np.array:
    """Compose transformed Pose covariance matrix, providing the covariance in
    the original frame, and the covariance of the transform.

    Args:
        covB (Union[list,tuple]): Pose covariance in the original frame
        aTb (TransformStamped): Transformation
        covT (Union[list,tuple], optional): Covariance of transformation. 
        Defaults to np.zeros([6,6]).

    Returns:
        np.array: Pose covariance in the transformed frame.
    """
    Sigma1 = np.array(covB).reshape([6,6])
    rquat = aTb.transform.rotation
    trans = aTb.transform.translation
    R = Rotation.from_quat([rquat.x, rquat.y, rquat.z, rquat.w])
    Sigma2 = np.array(covT).reshape([6,6])
    Dn = np.array([[0., -trans.z, trans.y],
                    [trans.z, 0., -trans.x],
                    [-trans.y, trans.x, 0.]])
    J = np.array([[R, np.zeros([3,3])], [np.dot(Dn, R), R]])
    covA = J @ Sigma1 @ J.transpose() + Sigma2
    return covA

def composeRotCov(covB:Union[list,tuple], aRb:TransformStamped,
                    covR:Union[list,tuple]=None)->np.array:
    """Compose transformed covariance of a Vector, providing the covariance in
    the original frame and the covariance of the transform.

    Args:
        covB (Union[list,tuple]): Vector covariance in the original frame.
        aRb (TransformStamped): Transformation.
        covR (Union[list,tuple], optional): Covariance of transformation.
        Defaults to None.

    Returns:
        np.array: Vector covariance in the transform frame.
    """
    Sigma1 = np.array(covB).reshape([6,6])
    rquat = aRb.transform.rotation
    R = Rotation.from_quat([rquat.x, rquat.y, rquat.z, rquat.w])
    J = np.array([[R, np.zeros([3,3])], [np.zeros([3,3]), R]])
    covA = J @ Sigma1 @ J.transpose()
    if covR:
        covA += np.array(covR).reshape([6,6])
    return covA

def tortuosity(rowVectors:Union[list,np.array], granularity:float=None)->float:
    """Computes the path tortuosity of the given time series as an array, under
    dimension (log-scaling) definition.

    Args:
        rowVectors (Union[list,np.array]): Time series array
        granularity (float, optional): Characteristic scale that distinguishes
        signal from noise. Defaults to None.

    Returns:
        float: Tortuosity of the given time series.
    """
    rowVectors = np.array(rowVectors)
    if np.ndim(rowVectors)<=1:
        return 0
    d = np.diff(rowVectors, axis=0)
    totalLength = np.sqrt(np.sum(d*d))
    nStep = d.shape[0]
    if nStep <= 1:
        return 0.
    if not granularity:
        nLargerStep = 1
        displacement = np.linalg.norm(rowVectors[-1,:]-rowVectors[0,:])
    else:
        nLargerStep = 0
        displacement = 0.
        granSq = granularity * granularity
        start = rowVectors[0]
        for i in rowVectors:
            diff = i - start
            norm = diff @ diff
            if norm < granSq:
                continue
            start = i
            nLargerStep += 1
            displacement += np.sqrt(norm)
        if nLargerStep == 0:
            nLargerStep = 1                 # to avoid singularity
    return np.log(totalLength/displacement)/np.log(nStep/nLargerStep)

def covToTolerance(cov:Union[list,tuple], decoupling:bool=True,
                    averaging:bool=True)->tuple:
    """Convert covariance matrix to directional tolerance

    Args:
        cov (Union[list,tuple]): Covariance matrix
        decoupling (bool, optional): Consider Sigma(0:3, 0:3) and 
        Sigma(3:6,3:6) only. Defaults to True.
        averaging (bool, optional): Averaging within linear and angular
        dimensions. Defaults to True.

    Returns:
        tuple: (Tolerance, Directions)
    """
    sigma = np.reshape(cov, [6,6])
    # numpy.linalg.eig returns normalized eigen vectors by default.
    if decoupling:
        eigvalL, eigvecL = np.linalg.eig(sigma[0:3, 0:3])
        eigvalA, eigvecA = np.linalg.eig(sigma[3:6, 3:6])
        if averaging:
            """homogenizing the linear and angular tolerances respectively"""
            tolTmp = [np.average(eigvalL[eigvalL>0]),
                        np.average(eigvalA[eigvalA>0])]
            tol = [tolTmp[0], tolTmp[0], tolTmp[0],
                    tolTmp[1], tolTmp[1], tolTmp[1]]
            dir = np.eye(6)
            return np.sqrt(tol), dir
        else:
            tol = [eigvalL[0], eigvalL[1], eigvalL[2],
                    eigvalA[0], eigvalA[1], eigvalA[2]]
            dir = np.block([[eigvecL,np.zeros([3,3])],
                            [np.zeros(3,3), eigvecA]])
            return np.sqrt(tol), dir
    else:
        eigval, eigvec = np.linalg.eig(sigma)
        return np.sqrt(eigval), eigvec

def toleranceToCov(tol, axes:np.array=np.eye(6), normalizedAxes:bool=True,
                    reshape:bool=True)->np.array:
    """Coverts directional tolerance to covariance matrix

    Args:
        tol (_type_): Tolerance on each dimensions
        axes (np.array, optional): Tolerance directions. Defaults to np.eye(6).
        normalizedAxes (bool, optional): Directions are normalized. Defaults
        to True.
        reshape (bool, optional): Flatten output to 1D. Defaults to True.

    Raises:
        RuntimeError: Tolerance shape is not 1 (uniform for linear and angular)
        or 2 (uniform within linear and angular) or 6 (all 6 DoF heterogeneous)

    Returns:
        np.array: Flattened or unflattened covariance matrix.
    """
    if type(tol)==float:
        tolL = [tol, tol, tol, tol, tol, tol]
    elif len(tol)==2:
        tolL = [tol[0], tol[0], tol[0], tol[1], tol[1], tol[1]]
    elif len(tol)==6:
        tolL = tol
    else:
        raise RuntimeError('Shape of tolerance is not 1/2/6 -- Unrecognized!')
    lam = np.diag(np.array(tolL)**2)
    if not normalizedAxes:
        # hopefully we are only throwing away a diagonal matrix here...
        basis, _ = np.linalg.qr(np.reshape(axes, [6,6]))
    else:
        basis = np.array(axes).reshape([6,6])
    cov = basis @ lam @ basis.transpose()
    if reshape:
        return cov.flatten()
    else:
        return cov

def updateTransform(transform:TransformStamped, tfBuffer:tf2_ros.Buffer,
                    timeout:float=0)->tuple:
    """Look within the transform buffer and update a guessed transform

    Args:
        transform (TransformStamped): Guessed transform
        tfBuffer (tf2_ros.Buffer): Transform buffer
        timeout (float, optional): Timeout in sec. Defaults to 0 (no timeout).

    Returns:
        tuple: (Transform, whether update was successful)
    """
    if not tfBuffer.can_transform(transform.child_frame_id, 
                                transform.header.frame_id,
                                rospy.Time(0), rospy.Duration(timeout)):
        return transform, False
    # assembling world pose from filtered odom msg and map pose
    newTransform:TransformStamped = tfBuffer.lookup_transform(
                        transform.child_frame_id, transform.header.frame_id,
                        rospy.Time(0))
    if newTransform.header.stamp < transform.header.stamp:
        # time stamp is not updated. assume the newer one is valid
        return transform, False
    return newTransform, True
