#!/usr/bin/python

import numpy as np
import copy
from geometry_msgs.msg import TransformStamped, PoseStamped
from scipy.spatial.transform import Rotation
import rospy
import tf2_geometry_msgs

def angleDiff(ang1, ang2):
    return np.mod((ang1 - ang2 + np.pi), np.pi+np.pi) - np.pi

def rpyFromQuaternion(quat):
    return Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')

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

def composeRotCov(covA, aRb, covR=None):
    Sigma1 = np.array(covA).reshape([6,6])
    rquat = aRb.transform.rotation
    R = Rotation.from_quat([rquat.x, rquat.y, rquat.z, rquat.w])
    J = np.array([[R, np.zeros([3,3])], [np.zeros([3,3]), R]]).transpose()
    covB = J @ Sigma1 @ J.transpose()
    if covR:
        covB += np.array(covR).reshape([6,6])
    return covB

def tortuosity(rowVectors, granularity=None):
    if np.ndim(rowVectors)<=1:
        return 0
    rowVectors = np.array(rowVectors)
    d = np.diff(rowVectors, axis=0)
    totalLength = np.sqrt(np.sum(d*d))
    nStep = d.shape[0]
    if not granularity:
        nLargerStep = 1
        displacement = rowVectors[-1,:]-rowVectors[0,:]
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

def covToTolerance(cov, decoupling=True, averaging=True):
    sigma = np.reshape(cov, [6,6])
    # numpy.linalg.eig returns normalized eigen vectors by default.
    if decoupling:
        eigvalL, eigvecL = np.linalg.eig(sigma[0:3, 0:3])
        eigvalA, eigvecA = np.linalg.eig(sigma[3:6, 3:6])
        if averaging:
            tolTmp = [np.average(eigvalL[eigvalL>0]), np.average(eigvalA[eigvalA>0])]
            tol = [tolTmp[0], tolTmp[0], tolTmp[0], tolTmp[1], tolTmp[1], tolTmp[1]]
            dir = np.eye(6)
            return np.sqrt(tol), dir
        else:
            tol = [eigvalL[0], eigvalL[1], eigvalL[2], eigvalA[0], eigvalA[1], eigvalA[2]]
            dir = np.block([[eigvecL,np.zeros([3,3])], [np.zeros(3,3), eigvecA]])
            return np.sqrt(tol), dir
    else:
        eigval, eigvec = np.linalg.eig(sigma)
        return np.sqrt(eigval), eigvec

def toleranceToCov(tol, axes=np.eye(6), normalizedAxes=True, reshape=True):
    lam = np.diag(np.array(tol)**2)
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

def updateTransform(transform, tfBuffer, timeout=0):
    if not tfBuffer.can_transform(transform.child_frame_id, 
                                transform.header.frame_id,
                                rospy.Time(0), rospy.Duration(timeout)):
        return transform, False
    # assembling world pose from filtered odom msg and map pose
    newTransform = tfBuffer.lookup_transform(transform.child_frame_id, 
                                            transform.header.frame_id,
                                            rospy.Time(0))
    if newTransform.header.stamp < transform.header.stamp:
        # time stamp is not updated. assume the newer one is valid
        return transform, False
    return newTransform, True
