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
import math
import GPy
#from pypointmatcher import pointmatcher as pm, pointmatchersupport as pms
#PM = pm.PointMatcher
#DP = PM.DataPoints

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

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

def check_if_in_interval(list_trajectories_split, size_interval, time_trimble_1, time_trimble_2, time_trimble_3, trimble_1, trimble_2, trimble_3, target_time):
    isInInterval = False
    time_1 = []
    time_2 = []
    time_3 = []
    traj_1 = []
    traj_2 = []
    traj_3 = []
    for j in list_trajectories_split:
        if (j[:, 0][1] - j[:, 0][0] > size_interval and j[:, 1][1] - j[:, 1][0] > size_interval
                and j[:, 2][1] - j[:, 2][0] > size_interval):
            interval_1 = j[:, 0]
            interval_2 = j[:, 1]
            interval_3 = j[:, 2]
            time_1 = time_trimble_1[interval_1[0]:interval_1[1]+1]
            time_2 = time_trimble_2[interval_2[0]:interval_2[1]+1]
            time_3 = time_trimble_3[interval_3[0]:interval_3[1]+1]
            traj_1 = trimble_1[0:3, interval_1[0]:interval_1[1]+1]
            traj_2 = trimble_2[0:3, interval_2[0]:interval_2[1]+1]
            traj_3 = trimble_3[0:3, interval_3[0]:interval_3[1]+1]
            min_interval = max([time_1[0], time_2[0], time_3[0]], key=lambda x: float(x))
            max_interval = min([time_1[-1], time_2[-1], time_3[-1]], key=lambda x: float(x))
            if (target_time <= max_interval and target_time >= min_interval):
                isInInterval = True
                break
    return isInInterval, time_1, time_2, time_3, traj_1, traj_2, traj_3

def define_window_training(time_1, time_2, time_3, window, target_time):
    close_1 = find_nearest(time_1, target_time)
    close_2 = find_nearest(time_2, target_time)
    close_3 = find_nearest(time_3, target_time)

    if close_1 - math.floor(window / 2) >= 0:
        if len(time_1) - close_1 - 1 - math.floor(window / 2) > 0:
            index_1 = np.array([close_1 - math.floor(window / 2), close_1 + math.floor(window / 2) - 1])
        else:
            index_1 = np.array([len(time_1) - window, len(time_1) - 1])
    else:
        index_1 = np.array([0, window - 1])

    if close_2 - window / 2 >= 0:
        if len(time_2) - close_2 - math.floor(window / 2) - 1 > 0:
            index_2 = np.array([close_2 - math.floor(window / 2), close_2 + math.floor(window / 2) - 1])
        else:
            index_2 = np.array([len(time_2) - window, len(time_2) - 1])
    else:
        index_2 = np.array([0, window - 1])

    if close_3 - math.floor(window / 2) >= 0:
        if len(time_3) - close_3 - 1 - window / 2 > 0:
            index_3 = np.array([close_3 - math.floor(window / 2), close_3 + math.floor(window / 2) - 1])
        else:
            index_3 = np.array([len(time_3) - window, len(time_3) - 1])
    else:
        index_3 = np.array([0, window - 1])

    return index_1, index_2, index_3

def define_time_window_training(time_1, time_2, time_3, window, target_time, limit_p, mode):
    min_1 = 0
    min_2 = 0
    min_3 = 0
    max_1 = 0
    max_2 = 0
    max_3 = 0
    if(mode == "ground-truth"):
        min_1 = find_nearest(time_1, target_time - window / 2)
        min_2 = find_nearest(time_2, target_time - window / 2)
        min_3 = find_nearest(time_3, target_time - window / 2)
        max_1 = find_nearest(time_1, target_time + window / 2)
        max_2 = find_nearest(time_2, target_time + window / 2)
        max_3 = find_nearest(time_3, target_time + window / 2)
    if(mode == "prediction"):
        min_1 = find_nearest(time_1, target_time - window )
        min_2 = find_nearest(time_2, target_time - window )
        min_3 = find_nearest(time_3, target_time - window )
        max_1 = find_nearest(time_1, target_time)
        max_2 = find_nearest(time_2, target_time)
        max_3 = find_nearest(time_3, target_time)
    size_1 = len(time_1[min_1:max_1])
    size_2 = len(time_2[min_2:max_2])
    size_3 = len(time_3[min_3:max_3])
    if((size_1>limit_p[0] and size_2>limit_p[0] and size_3>limit_p[1]) or
            (size_1>limit_p[0] and size_3>limit_p[0] and size_2>limit_p[1]) or
            (size_3>limit_p[0] and size_2>limit_p[0] and size_1>limit_p[1])):
        index_1 = np.array([min_1, max_1])
        index_2 = np.array([min_2, max_2])
        index_3 = np.array([min_3, max_3])
    else:
        index_1 = np.array([-1, -1])
        index_2 = np.array([-1, -1])
        index_3 = np.array([-1, -1])
    return index_1, index_2, index_3

def data_training_MGPO(time_1, time_2, time_3, traj_1, traj_2, traj_3, index_1, index_2, index_3):
    T_1_train_MGPO = np.atleast_2d(time_1[index_1[0]:index_1[1]+1]).T
    X_1_train_MGPO = np.atleast_2d(traj_1[0, index_1[0]:index_1[1]+1]).T
    Y_1_train_MGPO = np.atleast_2d(traj_1[1, index_1[0]:index_1[1]+1]).T
    Z_1_train_MGPO = np.atleast_2d(traj_1[2, index_1[0]:index_1[1]+1]).T

    T_2_train_MGPO = np.atleast_2d(time_2[index_2[0]:index_2[1]+1]).T
    X_2_train_MGPO = np.atleast_2d(traj_2[0, index_2[0]:index_2[1]+1]).T
    Y_2_train_MGPO = np.atleast_2d(traj_2[1, index_2[0]:index_2[1]+1]).T
    Z_2_train_MGPO = np.atleast_2d(traj_2[2, index_2[0]:index_2[1]+1]).T

    T_3_train_MGPO = np.atleast_2d(time_3[index_3[0]:index_3[1]+1]).T
    X_3_train_MGPO = np.atleast_2d(traj_3[0, index_3[0]:index_3[1]+1]).T
    Y_3_train_MGPO = np.atleast_2d(traj_3[1, index_3[0]:index_3[1]+1]).T
    Z_3_train_MGPO = np.atleast_2d(traj_3[2, index_3[0]:index_3[1]+1]).T

    T_MGPO = [T_1_train_MGPO, T_1_train_MGPO, T_1_train_MGPO, T_2_train_MGPO, T_2_train_MGPO, T_2_train_MGPO,
              T_3_train_MGPO, T_3_train_MGPO,
              T_3_train_MGPO]
    S_MGPO = [X_1_train_MGPO, Y_1_train_MGPO, Z_1_train_MGPO, X_2_train_MGPO, Y_2_train_MGPO, Z_2_train_MGPO,
              X_3_train_MGPO, Y_3_train_MGPO, Z_3_train_MGPO]

    return T_MGPO, S_MGPO

def data_training_MGPO_raw(time_1, time_2, time_3, d_1, a_1, e_1, d_2, a_2, e_2, d_3, a_3, e_3, index_1, index_2, index_3):
    T_1_train_MGPO = np.atleast_2d(time_1[index_1[0]:index_1[1]+1]).T
    X_1_train_MGPO = np.atleast_2d(d_1[index_1[0]:index_1[1]+1]).T
    Y_1_train_MGPO = np.atleast_2d(a_1[index_1[0]:index_1[1]+1]).T
    Z_1_train_MGPO = np.atleast_2d(e_1[index_1[0]:index_1[1]+1]).T

    T_2_train_MGPO = np.atleast_2d(time_2[index_2[0]:index_2[1]+1]).T
    X_2_train_MGPO = np.atleast_2d(d_2[index_2[0]:index_2[1]+1]).T
    Y_2_train_MGPO = np.atleast_2d(a_2[index_2[0]:index_2[1]+1]).T
    Z_2_train_MGPO = np.atleast_2d(e_2[index_2[0]:index_2[1]+1]).T

    T_3_train_MGPO = np.atleast_2d(time_3[index_3[0]:index_3[1]+1]).T
    X_3_train_MGPO = np.atleast_2d(d_3[index_3[0]:index_3[1]+1]).T
    Y_3_train_MGPO = np.atleast_2d(a_3[index_3[0]:index_3[1]+1]).T
    Z_3_train_MGPO = np.atleast_2d(e_3[index_3[0]:index_3[1]+1]).T

    T_MGPO = [T_1_train_MGPO, T_1_train_MGPO, T_1_train_MGPO, T_2_train_MGPO, T_2_train_MGPO, T_2_train_MGPO,
              T_3_train_MGPO, T_3_train_MGPO,
              T_3_train_MGPO]
    S_MGPO = [X_1_train_MGPO, Y_1_train_MGPO, Z_1_train_MGPO, X_2_train_MGPO, Y_2_train_MGPO, Z_2_train_MGPO,
              X_3_train_MGPO, Y_3_train_MGPO, Z_3_train_MGPO]

    return T_MGPO, S_MGPO

def data_training_GP(time_1, time_2, time_3, traj_1, traj_2, traj_3, index_1, index_2, index_3):
    T_1_train_GP = np.atleast_2d(time_1[index_1[0]:index_1[1]+1]).T
    X_1_train_GP = np.atleast_2d(traj_1[0, index_1[0]:index_1[1]+1]).T
    Y_1_train_GP = np.atleast_2d(traj_1[1, index_1[0]:index_1[1]+1]).T
    Z_1_train_GP = np.atleast_2d(traj_1[2, index_1[0]:index_1[1]+1]).T
    T_2_train_GP = np.atleast_2d(time_2[index_2[0]:index_2[1]+1]).T
    X_2_train_GP = np.atleast_2d(traj_2[0, index_2[0]:index_2[1]+1]).T
    Y_2_train_GP = np.atleast_2d(traj_2[1, index_2[0]:index_2[1]+1]).T
    Z_2_train_GP = np.atleast_2d(traj_2[2, index_2[0]:index_2[1]+1]).T
    T_3_train_GP = np.atleast_2d(time_3[index_3[0]:index_3[1]+1]).T
    X_3_train_GP = np.atleast_2d(traj_3[0, index_3[0]:index_3[1]+1]).T
    Y_3_train_GP = np.atleast_2d(traj_3[1, index_3[0]:index_3[1]+1]).T
    Z_3_train_GP = np.atleast_2d(traj_3[2, index_3[0]:index_3[1]+1]).T

    return T_1_train_GP, X_1_train_GP, Y_1_train_GP, Z_1_train_GP, T_2_train_GP, X_2_train_GP, Y_2_train_GP, Z_2_train_GP, T_3_train_GP, X_3_train_GP, Y_3_train_GP, Z_3_train_GP

def data_training_GP_raw(time_1, time_2, time_3, d_1, a_1, e_1, d_2, a_2, e_2, d_3, a_3, e_3, index_1, index_2, index_3):
    T_1_train_GP = np.atleast_2d(time_1[index_1[0]:index_1[1]+1]).T
    X_1_train_GP = np.atleast_2d(d_1[0, index_1[0]:index_1[1]+1]).T
    Y_1_train_GP = np.atleast_2d(a_1[1, index_1[0]:index_1[1]+1]).T
    Z_1_train_GP = np.atleast_2d(e_1[2, index_1[0]:index_1[1]+1]).T
    T_2_train_GP = np.atleast_2d(time_2[index_2[0]:index_2[1]+1]).T
    X_2_train_GP = np.atleast_2d(d_2[0, index_2[0]:index_2[1]+1]).T
    Y_2_train_GP = np.atleast_2d(a_2[1, index_2[0]:index_2[1]+1]).T
    Z_2_train_GP = np.atleast_2d(e_2[2, index_2[0]:index_2[1]+1]).T
    T_3_train_GP = np.atleast_2d(time_3[index_3[0]:index_3[1]+1]).T
    X_3_train_GP = np.atleast_2d(d_3[0, index_3[0]:index_3[1]+1]).T
    Y_3_train_GP = np.atleast_2d(a_3[1, index_3[0]:index_3[1]+1]).T
    Z_3_train_GP = np.atleast_2d(e_3[2, index_3[0]:index_3[1]+1]).T

    return T_1_train_GP, X_1_train_GP, Y_1_train_GP, Z_1_train_GP, T_2_train_GP, X_2_train_GP, Y_2_train_GP, Z_2_train_GP, T_3_train_GP, X_3_train_GP, Y_3_train_GP, Z_3_train_GP

def training_MGPO(num_restarts, verbose, T_MGPO, S_MGPO):
    input_dim = 1
    #variance = 0.002
    #variance_constraint = 1
    #lengthscale = 1.
    #K = GPy.kern.Matern52(input_dim=input_dim, variance=variance, lengthscale=lengthscale)
    K = GPy.kern.Matern52(input_dim=input_dim, ARD=True)
    icm = GPy.util.multioutput.ICM(input_dim=1, num_outputs=9, kernel=K)
    m = GPy.models.GPCoregionalizedRegression(T_MGPO, S_MGPO, kernel=icm)
    #m['.*Mat52.var'].constrain_fixed(variance_constraint)
    m.optimize(messages=verbose)
    m.optimize_restarts(num_restarts=num_restarts, verbose=verbose)
    return m

def training_GP(num_restarts, verbose, T_1_train_GP, X_1_train_GP, Y_1_train_GP, Z_1_train_GP, T_2_train_GP, X_2_train_GP, Y_2_train_GP, Z_2_train_GP, T_3_train_GP, X_3_train_GP, Y_3_train_GP, Z_3_train_GP):
    input_dim = 1
    #variance = 0
    #variance_constraint = 1000
    lengthscale = 1.

    #kx1 = GPy.kern.Matern52(input_dim=input_dim, ARD=True) + GPy.kern.Bias(input_dim)
    kx1 = GPy.kern.Matern52(input_dim=input_dim, ARD=True)
    m_x1 = GPy.models.GPRegression(T_1_train_GP, X_1_train_GP, kx1)
    #m_x1['.*Mat52.var'].constrain_fixed(variance_constraint)
    #m_x1.unconstrain()
    m_x1.optimize(messages=verbose)
    m_x1.optimize_restarts(num_restarts=num_restarts, verbose=verbose)

    ky1 = GPy.kern.Matern52(input_dim=input_dim, ARD=True)
    m_y1 = GPy.models.GPRegression(T_1_train_GP, Y_1_train_GP, ky1)
    #m_y1.unconstrain()
    #m_y1['.*Mat52.var'].constrain_fixed(variance_constraint)
    m_y1.optimize(messages=verbose)
    m_y1.optimize_restarts(num_restarts=num_restarts, verbose=verbose)

    kz1 = GPy.kern.Matern52(input_dim=input_dim, ARD=True)
    m_z1 = GPy.models.GPRegression(T_1_train_GP, Z_1_train_GP, kz1)
    #m_z1.unconstrain()
    m_z1.optimize(messages=verbose)
    #m_z1['.*Mat52.var'].constrain_fixed(variance_constraint)
    m_z1.optimize_restarts(num_restarts=num_restarts, verbose=verbose)

    kx2 = GPy.kern.Matern52(input_dim=input_dim, ARD=True)
    m_x2 = GPy.models.GPRegression(T_2_train_GP, X_2_train_GP, kx2)
    #m_x2.unconstrain()
    #m_x2['.*Mat52.var'].constrain_fixed(variance_constraint)
    m_x2.optimize(messages=verbose)
    m_x2.optimize_restarts(num_restarts=num_restarts, verbose=verbose)

    ky2 = GPy.kern.Matern52(input_dim=input_dim, ARD=True)
    m_y2 = GPy.models.GPRegression(T_2_train_GP, Y_2_train_GP, ky2)
    #m_y2.unconstrain()
    #m_y2['.*Mat52.var'].constrain_fixed(variance_constraint)
    m_y2.optimize(messages=verbose)
    m_y2.optimize_restarts(num_restarts=num_restarts, verbose=verbose)

    kz2 = GPy.kern.Matern52(input_dim=input_dim, ARD=True)
    m_z2 = GPy.models.GPRegression(T_2_train_GP, Z_2_train_GP, kz2)
    #m_z2.unconstrain()
    #m_z2['.*Mat52.var'].constrain_fixed(variance_constraint)
    m_z2.optimize(messages=verbose)
    m_z2.optimize_restarts(num_restarts=num_restarts, verbose=verbose)

    kx3 = GPy.kern.Matern52(input_dim=input_dim, ARD=True)
    m_x3 = GPy.models.GPRegression(T_3_train_GP, X_3_train_GP, kx3)
    #m_x3.unconstrain()
    #m_x3['.*Mat52.var'].constrain_fixed(variance_constraint)
    m_x3.optimize(messages=verbose)
    m_x3.optimize_restarts(num_restarts=num_restarts, verbose=verbose)

    ky3 = GPy.kern.Matern52(input_dim=input_dim, ARD=True)
    m_y3 = GPy.models.GPRegression(T_3_train_GP, Y_3_train_GP, ky3)
    #m_y3.unconstrain()
    #m_y3['.*Mat52.var'].constrain_fixed(variance_constraint)
    m_y3.optimize(messages=verbose)
    m_y3.optimize_restarts(num_restarts=num_restarts, verbose=verbose)

    kz3 = GPy.kern.Matern52(input_dim=input_dim, ARD=True)
    m_z3 = GPy.models.GPRegression(T_3_train_GP, Z_3_train_GP, kz3)
    #m_z3.unconstrain()
    #m_z3['.*Mat52.var'].constrain_fixed(variance_constraint)
    m_z3.optimize(messages=verbose)
    m_z3.optimize_restarts(num_restarts=num_restarts, verbose=verbose)

    return m_x1, m_y1, m_z1, m_x2, m_y2, m_z2, m_x3, m_y3, m_z3

def unit_prediction_MGPO(timestamp, m):
    new_Time_MGPO = np.array([timestamp, 1])[:, None]
    A_MGPO = np.ones_like(new_Time_MGPO)
    A_MGPO.fill(0)
    new_Time_x1_MGPO = np.hstack([new_Time_MGPO, A_MGPO])
    A_MGPO.fill(1)
    new_Time_y1_MGPO = np.hstack([new_Time_MGPO, A_MGPO])
    A_MGPO.fill(2)
    new_Time_z1_MGPO = np.hstack([new_Time_MGPO, A_MGPO])
    A_MGPO.fill(3)
    new_Time_x2_MGPO = np.hstack([new_Time_MGPO, A_MGPO])
    A_MGPO.fill(4)
    new_Time_y2_MGPO = np.hstack([new_Time_MGPO, A_MGPO])
    A_MGPO.fill(5)
    new_Time_z2_MGPO = np.hstack([new_Time_MGPO, A_MGPO])
    A_MGPO.fill(6)
    new_Time_x3_MGPO = np.hstack([new_Time_MGPO, A_MGPO])
    A_MGPO.fill(7)
    new_Time_y3_MGPO = np.hstack([new_Time_MGPO, A_MGPO])
    A_MGPO.fill(8)
    new_Time_z3_MGPO = np.hstack([new_Time_MGPO, A_MGPO])

    noise_dict = {'output_index': new_Time_x1_MGPO[:, 1:].astype(int)}
    x1_pred_MGPO, sx1_pred_MGPO = m.predict_noiseless(new_Time_x1_MGPO, Y_metadata=noise_dict)
    noise_dict = {'output_index': new_Time_y1_MGPO[:, 1:].astype(int)}
    y1_pred_MGPO, sy1_pred_MGPO = m.predict_noiseless(new_Time_y1_MGPO, Y_metadata=noise_dict)
    noise_dict = {'output_index': new_Time_z1_MGPO[:, 1:].astype(int)}
    z1_pred_MGPO, sz1_pred_MGPO = m.predict_noiseless(new_Time_z1_MGPO, Y_metadata=noise_dict)
    noise_dict = {'output_index': new_Time_x2_MGPO[:, 1:].astype(int)}
    x2_pred_MGPO, sx2_pred_MGPO = m.predict_noiseless(new_Time_x2_MGPO, Y_metadata=noise_dict)
    noise_dict = {'output_index': new_Time_y2_MGPO[:, 1:].astype(int)}
    y2_pred_MGPO, sy2_pred_MGPO = m.predict_noiseless(new_Time_y2_MGPO, Y_metadata=noise_dict)
    noise_dict = {'output_index': new_Time_z2_MGPO[:, 1:].astype(int)}
    z2_pred_MGPO, sz2_pred_MGPO = m.predict_noiseless(new_Time_z2_MGPO, Y_metadata=noise_dict)
    noise_dict = {'output_index': new_Time_x3_MGPO[:, 1:].astype(int)}
    x3_pred_MGPO, sx3_pred_MGPO = m.predict_noiseless(new_Time_x3_MGPO, Y_metadata=noise_dict)
    noise_dict = {'output_index': new_Time_y3_MGPO[:, 1:].astype(int)}
    y3_pred_MGPO, sy3_pred_MGPO = m.predict_noiseless(new_Time_y3_MGPO, Y_metadata=noise_dict)
    noise_dict = {'output_index': new_Time_z3_MGPO[:, 1:].astype(int)}
    z3_pred_MGPO, sz3_pred_MGPO = m.predict_noiseless(new_Time_z3_MGPO, Y_metadata=noise_dict)

    x1_pred_MGPO = x1_pred_MGPO.flatten()
    x2_pred_MGPO = x2_pred_MGPO.flatten()
    x3_pred_MGPO = x3_pred_MGPO.flatten()
    y1_pred_MGPO = y1_pred_MGPO.flatten()
    y2_pred_MGPO = y2_pred_MGPO.flatten()
    y3_pred_MGPO = y3_pred_MGPO.flatten()
    z1_pred_MGPO = z1_pred_MGPO.flatten()
    z2_pred_MGPO = z2_pred_MGPO.flatten()
    z3_pred_MGPO = z3_pred_MGPO.flatten()
    sx1_pred_MGPO = sx1_pred_MGPO.flatten()
    sx2_pred_MGPO = sx2_pred_MGPO.flatten()
    sx3_pred_MGPO = sx3_pred_MGPO.flatten()
    sy1_pred_MGPO = sy1_pred_MGPO.flatten()
    sy2_pred_MGPO = sy2_pred_MGPO.flatten()
    sy3_pred_MGPO = sy3_pred_MGPO.flatten()
    sz1_pred_MGPO = sz1_pred_MGPO.flatten()
    sz2_pred_MGPO = sz2_pred_MGPO.flatten()
    sz3_pred_MGPO = sz3_pred_MGPO.flatten()

    P1 = np.array([timestamp, x1_pred_MGPO[0], y1_pred_MGPO[0], z1_pred_MGPO[0], sx1_pred_MGPO[0], sy1_pred_MGPO[0], sz1_pred_MGPO[0]])
    P2 = np.array([timestamp, x2_pred_MGPO[0], y2_pred_MGPO[0], z2_pred_MGPO[0], sx2_pred_MGPO[0], sy2_pred_MGPO[0], sz2_pred_MGPO[0]])
    P3 = np.array([timestamp, x3_pred_MGPO[0], y3_pred_MGPO[0], z3_pred_MGPO[0], sx3_pred_MGPO[0], sy3_pred_MGPO[0], sz3_pred_MGPO[0]])

    return P1, P2, P3

def unit_prediction_GP(timestamp, m_x1, m_y1, m_z1, m_x2, m_y2, m_z2, m_x3, m_y3, m_z3):
    new_Time_GP = np.array([timestamp, 1])[:, None]

    x1_pred_GP, sx1_pred_GP = m_x1.predict(new_Time_GP)
    y1_pred_GP, sy1_pred_GP = m_y1.predict(new_Time_GP)
    z1_pred_GP, sz1_pred_GP = m_z1.predict(new_Time_GP)
    x2_pred_GP, sx2_pred_GP = m_x2.predict(new_Time_GP)
    y2_pred_GP, sy2_pred_GP = m_y2.predict(new_Time_GP)
    z2_pred_GP, sz2_pred_GP = m_z2.predict(new_Time_GP)
    x3_pred_GP, sx3_pred_GP = m_x3.predict(new_Time_GP)
    y3_pred_GP, sy3_pred_GP = m_y3.predict(new_Time_GP)
    z3_pred_GP, sz3_pred_GP = m_z3.predict(new_Time_GP)

    x1_pred_GP = x1_pred_GP.flatten()
    x2_pred_GP = x2_pred_GP.flatten()
    x3_pred_GP = x3_pred_GP.flatten()
    y1_pred_GP = y1_pred_GP.flatten()
    y2_pred_GP = y2_pred_GP.flatten()
    y3_pred_GP = y3_pred_GP.flatten()
    z1_pred_GP = z1_pred_GP.flatten()
    z2_pred_GP = z2_pred_GP.flatten()
    z3_pred_GP = z3_pred_GP.flatten()
    sx1_pred_GP = sx1_pred_GP.flatten()
    sx2_pred_GP = sx2_pred_GP.flatten()
    sx3_pred_GP = sx3_pred_GP.flatten()
    sy1_pred_GP = sy1_pred_GP.flatten()
    sy2_pred_GP = sy2_pred_GP.flatten()
    sy3_pred_GP = sy3_pred_GP.flatten()
    sz1_pred_GP = sz1_pred_GP.flatten()
    sz2_pred_GP = sz2_pred_GP.flatten()
    sz3_pred_GP = sz3_pred_GP.flatten()

    P1 = np.array([timestamp, x1_pred_GP[0], y1_pred_GP[0], z1_pred_GP[0], sx1_pred_GP[0], sy1_pred_GP[0], sz1_pred_GP[0]])
    P2 = np.array([timestamp, x2_pred_GP[0], y2_pred_GP[0], z2_pred_GP[0], sx2_pred_GP[0], sy2_pred_GP[0], sz2_pred_GP[0]])
    P3 = np.array([timestamp, x3_pred_GP[0], y3_pred_GP[0], z3_pred_GP[0], sx3_pred_GP[0], sy3_pred_GP[0], sz3_pred_GP[0]])
    return P1, P2, P3

# error in mm
def error_calculation(Prediction_1, Prediction_2, Prediction_3, Dist_prism_12, Dist_prism_13, Dist_prism_23, D12_wref, D13_wref, D23_wref):
    error_euclidian_GP = []
    distance_wasserstein_GP = []
    error_wasserstein_diff_GP = []
    T_prediction = []
    for i, j, k in zip(Prediction_1, Prediction_2, Prediction_3):
        T_prediction.append(i[0])
        l = np.diag(np.array([i[4], i[5], i[6]]))
        m = np.diag(np.array([j[4], j[5], j[6]]))
        n = np.diag(np.array([k[4], k[5], k[6]]))
        d12 = abs(np.linalg.norm(i[1:4] - j[1:4]) * 1000 - Dist_prism_12 * 1000)
        d13 = abs(np.linalg.norm(i[1:4] - k[1:4]) * 1000 - Dist_prism_13 * 1000)
        d23 = abs(np.linalg.norm(j[1:4] - k[1:4]) * 1000 - Dist_prism_23 * 1000)
        md = np.mean([d12, d13, d23])
        error_euclidian_GP.append(np.array([d12, d13, d23, md]))
        d12_w = np.sqrt(np.linalg.norm(i[1:4] - j[1:4]) * np.linalg.norm(i[1:4] - j[1:4]) + (
                    l + m - 2 * np.sqrt(np.sqrt(l) @ m @ np.sqrt(l))).trace())
        d13_w = np.sqrt(np.linalg.norm(i[1:4] - k[1:4]) * np.linalg.norm(i[1:4] - k[1:4]) + (
                    l + n - 2 * np.sqrt(np.sqrt(l) @ n @ np.sqrt(l))).trace())
        d23_w = np.sqrt(np.linalg.norm(j[1:4] - k[1:4]) * np.linalg.norm(j[1:4] - k[1:4]) + (
                    m + n - 2 * np.sqrt(np.sqrt(m) @ n @ np.sqrt(m))).trace())
        diff_12w = abs(d12_w - D12_wref) * 1000
        diff_13w = abs(d13_w - D13_wref) * 1000
        diff_23w = abs(d23_w - D23_wref) * 1000
        distance_wasserstein_GP.append(np.array([d12_w, d13_w, d23_w]))
        error_wasserstein_diff_GP.append(np.array([diff_12w, diff_13w, diff_23w]))
    return error_euclidian_GP, distance_wasserstein_GP, error_wasserstein_diff_GP, T_prediction

def error_calculation_eucli(Prediction_1, Prediction_2, Prediction_3, Dist_prism_12, Dist_prism_13, Dist_prism_23):
    error_euclidian_GP = []
    T_prediction = []
    for i, j, k in zip(Prediction_1, Prediction_2, Prediction_3):
        T_prediction.append(i[0])
        d12 = abs(np.linalg.norm(i[1:4] - j[1:4]) - Dist_prism_12)*1000
        d13 = abs(np.linalg.norm(i[1:4] - k[1:4]) - Dist_prism_13)*1000
        d23 = abs(np.linalg.norm(j[1:4] - k[1:4]) - Dist_prism_23)*1000
        md = np.mean([d12, d13, d23])
        error_euclidian_GP.append(np.array([d12, d13, d23, md]))
    return error_euclidian_GP, T_prediction


import lab as B
import matplotlib.pyplot as plt
import torch
from wbml.plot import tweak
from stheno.torch import EQ, GP, Matern52

class Model_stheno(torch.nn.Module):
    """A GP model with learnable parameters."""
    def __init__(self, init_var=1, init_scale=1, init_noise=0):
        super().__init__()
        # Ensure that the parameters are positive and make them learnable.
        self.log_var = torch.nn.Parameter(torch.log(torch.tensor(init_var)))
        self.log_scale = torch.nn.Parameter(torch.log(torch.tensor(init_scale)))
        self.log_noise = torch.nn.Parameter(torch.log(torch.tensor(init_noise)))

    def construct(self):
        self.var = torch.exp(self.log_var)
        self.scale = torch.exp(self.log_scale)
        self.noise = torch.exp(self.log_noise)
        # kernel = self.var * EQ().stretch(self.scale)
        kernel = self.var * Matern52().stretch(self.scale)
        return GP(0, kernel), self.noise

def GP_function_stheno(x, x_obs, y_obs, variance, lengthscale, noise_init, optimization_nb):
    model = Model_stheno(init_var=variance, init_scale=lengthscale, init_noise=noise_init)
    f, noise = model.construct()
    # Condition on observations and make predictions before optimisation.
    f_post = f | (f(x_obs, noise), y_obs)
    prior_before = f, noise
    pred_before = f_post(x, noise).marginal_credible_bounds()
    # Perform optimisation.
    opt = torch.optim.Adam(model.parameters(), lr=1e-3)
    for _ in range(optimization_nb):
        opt.zero_grad()
        f, noise = model.construct()
        loss = -f(x_obs, noise).logpdf(y_obs)
        loss.backward()
        opt.step()
    f, noise = model.construct()
    # Condition on observations and make predictions after optimisation.
    f_post = f | (f(x_obs, noise), y_obs)
    prior_after = f, noise
    #mean, lower, upper = f_post(x, noise).marginal_credible_bounds()
    mean, variance = f_post(x, noise).marginals()
    return mean.detach().numpy().flatten(), variance.detach().numpy().flatten()

def data_training_GP_stheno(time_1, time_2, time_3, traj_1, traj_2, traj_3, index_1, index_2, index_3):
    T_1_train_GP = np.atleast_2d(time_1[index_1[0]:index_1[1]+1]).T
    X_1_train_GP = np.atleast_2d(traj_1[0, index_1[0]:index_1[1]+1]).T
    Y_1_train_GP = np.atleast_2d(traj_1[1, index_1[0]:index_1[1]+1]).T
    Z_1_train_GP = np.atleast_2d(traj_1[2, index_1[0]:index_1[1]+1]).T
    T_2_train_GP = np.atleast_2d(time_2[index_2[0]:index_2[1]+1]).T
    X_2_train_GP = np.atleast_2d(traj_2[0, index_2[0]:index_2[1]+1]).T
    Y_2_train_GP = np.atleast_2d(traj_2[1, index_2[0]:index_2[1]+1]).T
    Z_2_train_GP = np.atleast_2d(traj_2[2, index_2[0]:index_2[1]+1]).T
    T_3_train_GP = np.atleast_2d(time_3[index_3[0]:index_3[1]+1]).T
    X_3_train_GP = np.atleast_2d(traj_3[0, index_3[0]:index_3[1]+1]).T
    Y_3_train_GP = np.atleast_2d(traj_3[1, index_3[0]:index_3[1]+1]).T
    Z_3_train_GP = np.atleast_2d(traj_3[2, index_3[0]:index_3[1]+1]).T

    X_1_train_GP = torch.from_numpy(np.vstack(X_1_train_GP).astype(np.float).flatten())
    Y_1_train_GP = torch.from_numpy(np.vstack(Y_1_train_GP).astype(np.float).flatten())
    Z_1_train_GP = torch.from_numpy(np.vstack(Z_1_train_GP).astype(np.float).flatten())
    X_2_train_GP = torch.from_numpy(np.vstack(X_2_train_GP).astype(np.float).flatten())
    Y_2_train_GP = torch.from_numpy(np.vstack(Y_2_train_GP).astype(np.float).flatten())
    Z_2_train_GP = torch.from_numpy(np.vstack(Z_2_train_GP).astype(np.float).flatten())
    X_3_train_GP = torch.from_numpy(np.vstack(X_3_train_GP).astype(np.float).flatten())
    Y_3_train_GP = torch.from_numpy(np.vstack(Y_3_train_GP).astype(np.float).flatten())
    Z_3_train_GP = torch.from_numpy(np.vstack(Z_3_train_GP).astype(np.float).flatten())
    T_1_train_GP = torch.from_numpy(T_1_train_GP.flatten())
    T_2_train_GP = torch.from_numpy(T_2_train_GP.flatten())
    T_3_train_GP = torch.from_numpy(T_3_train_GP.flatten())

    return T_1_train_GP, X_1_train_GP, Y_1_train_GP, Z_1_train_GP, T_2_train_GP, X_2_train_GP, Y_2_train_GP, Z_2_train_GP, T_3_train_GP, X_3_train_GP, Y_3_train_GP, Z_3_train_GP
