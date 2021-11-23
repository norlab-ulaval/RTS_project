import numpy as np
from matplotlib import pyplot as plt
import random
import os
from matplotlib import animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy import spatial
from IPython.display import HTML
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import (RBF, Matern, RationalQuadratic, ExpSineSquared, DotProduct, ConstantKernel)
from pypointmatcher import pointmatcher as pm, pointmatchersupport as pms
PM = pm.PointMatcher
DP = PM.DataPoints

def ICP_init():
    Parameters = pms.Parametrizable.Parameters
    # Create the default ICP algrotithm
    icp = PM.ICP()
    params = Parameters()
    # Comment out to stop console outputs
    pms.setLogger(PM.get().LoggerRegistrar.create("FileLogger"))
    # Prepare matching function
    name = "OneIndexMatcher"
    kdtree = PM.get().MatcherRegistrar.create(name, params)
    params.clear()
    # Prepare error minimization
    name = "PointToPlaneErrorMinimizer"
    pointToPoint = PM.get().ErrorMinimizerRegistrar.create(name)
    params.clear()
    # Prepare transformation checker filters
    name = "CounterTransformationChecker"
    params["maxIterationCount"] = "30"
    maxIter = PM.get().TransformationCheckerRegistrar.create(name, params)
    params.clear()
    name = "DifferentialTransformationChecker"
    params["minDiffRotErr"] = "0.001"
    params["minDiffTransErr"] = "0.01"
    params["smoothLength"] = "4"
    diff = PM.get().TransformationCheckerRegistrar.create(name, params)
    params.clear()
    # Prepare inspector
    # Comment out to write vtk files per iteration
    name = "NullInspector"
    nullInspect = PM.get().InspectorRegistrar.create(name)
    # Prepare transformation
    name = "RigidTransformation"
    rigid_trans = PM.get().TransformationRegistrar.create(name)
    # Build ICP solution
    icp.matcher = kdtree
    icp.errorMinimizer = pointToPoint
    icp.transformationCheckers.append(maxIter)
    icp.transformationCheckers.append(diff)
    # Toggle to write vtk files per iteration
    icp.inspector = nullInspect
    icp.transformations.append(rigid_trans)
    return icp

def ICP_registration_Gaussian(icp, P, Q, W, N):
    # Express P and Q for point-to-Gaussian minimization
    Pg = np.array([P[:,0], P[:,0], P[:,0], P[:,1], P[:,1], P[:,1], P[:,2], P[:,2],  P[:,2]]).T
    Qg = np.array([Q[:,0], Q[:,0], Q[:,0], Q[:,1], Q[:,1], Q[:,1], Q[:,2], Q[:,2],  Q[:,2]]).T
    Qg_c = np.array([W[0,0]*N[0],W[1,0]*N[1],W[2,0]*N[2],
                     W[0,1]*N[0],W[1,1]*N[1],W[2,1]*N[2],
                     W[0,2]*N[0],W[1,2]*N[1],W[2,2]*N[2]]).T
    #TODO: change init
    ref = DP(DP.load('/home/maxime/Libraries/libpointmatcher/examples/data/car_cloud400.csv'))
    data = DP(DP.load('/home/maxime/Libraries/libpointmatcher/examples/data/car_cloud401.csv'))
    #ref = DP()
    #data = DP()
    #DP.addDescriptor(ref, 'normals', Qg_c)
    #DP.addFeature(ref, 'x, y, z, pad', Qg)
    #DP.addFeature(data, 'x, y, z, pad', Pg)
    data.features = Pg
    #data.featureLabels = 'x, y, z, pad'
    ref.features = Qg
    #ref.featureLabels = 'x, y, z, pad'
    ref.descriptors = Qg_c
    #ref.descriptorLabels = 'normals'
    # Compute the transformation to express data in ref
    T = icp(data, ref)
    # Transform data to express it in ref
    data_out = DP(data)
    icp.transformations.apply(data_out, T)
    return T

def init_GP(kernel, alpha_noise, restarts_optimizer):
	# Instantiate a Gaussian Process model
	kernel_model = [ConstantKernel(1.0, (1e-3, 1e3)) * RBF(length_scale=19, length_scale_bounds=(1e-2, 1e2)),   #1.0,1e-3,1e3,10,1e-2,1e2
									1.0 * RBF(length_scale=5, length_scale_bounds=(1e-1, 100.0)),  
									1.0 * RationalQuadratic(length_scale=1.0, alpha=0.1),
									1.0 * ExpSineSquared(length_scale=1.0, periodicity=3.0,length_scale_bounds=(0.1, 10.0), periodicity_bounds=(1.0, 10.0)),
									ConstantKernel(0.1, (0.01, 10.0))*(DotProduct(sigma_0=1.0, sigma_0_bounds=(0.1, 10.0)) ** 2),
									1.0 * Matern(length_scale=10.0, length_scale_bounds=(1e-2, 100.0), nu=2.5),
									1.0 * Matern(length_scale=10.0, length_scale_bounds=(1e-2, 100.0), nu=1.5)]
	kernel_used = kernel_model[kernel]
	# GP interpolation
	gp = GaussianProcessRegressor(kernel=kernel_used, alpha=alpha_noise, n_restarts_optimizer=restarts_optimizer, normalize_y=False)   #0.0002, 9
	return gp

def fit_GP(gp, T, X):
	gp.fit(T, X)
	return gp

def predict_GP(gp, T_prediction, return_std):
	if(return_std==True):
		x_pred, sigma = gp.predict(T_prediction, return_std=return_std)
		return x_pred, sigma
	else:
		x_pred = gp.predict(T_prediction, return_std=return_std)
		return x_pred

def GP_training(kernel, alpha_noise, restarts_optimizer, T, X):
	gp = init_GP(kernel, alpha_noise, restarts_optimizer)
	return fit_GP(gp, T, X)





















