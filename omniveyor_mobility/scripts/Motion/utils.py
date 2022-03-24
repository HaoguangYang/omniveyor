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

def tortuosity(rowVectors, granularity=None):
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
            if norm >= granSq:
                start = i
                nLargerStep += 1
                displacement += np.sqrt(norm)
        if nLargerStep == 0:
            nLargerStep = 1                 # to avoid singularity
    return np.log(totalLength/displacement)/np.log(nStep/nLargerStep)

def covToTolerance(cov, decoupling=True, averaging=True):
    sigma = np.reshape(cov, [6,6])
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
