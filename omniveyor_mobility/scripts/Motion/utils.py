#!/usr/bin/python

import numpy as np
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation

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