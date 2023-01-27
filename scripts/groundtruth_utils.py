import numpy as np
from scripts.theodolite_utils import *
from scripts.theodolite_function import *
from scripts.prediction_utils import *
from scripts.resection_functions import *

from pylgmath import so3op, se3op, Transformation
from pysteam.evaluable.se3 import SE3StateVar
from pysteam.evaluable.vspace import VSpaceStateVar
from pysteam.evaluable.p2p import P2PErrorEvaluator
from pysteam.trajectory import Time
from pysteam.trajectory.const_vel import Interface as TrajectoryInterface
from pysteam.problem import OptimizationProblem, StaticNoiseModel, L2LossFunc, WeightedLeastSquareCostTerm, DynamicNoiseModel
from pysteam.solver import GaussNewtonSolver
from pysteam.solver import Covariance

def pipeline_groundtruth(path, Sensor, path_sensor_file, parameters, file, output, path_sensor_file_synch_time, Gps_reference_chosen):

    sensor_data = []
    if Sensor == "GNSS":
        GNSS_raw_data = read_prediction_data_Linear_csv_file(path_sensor_file)
        time_delay = float(read_time_delay(path_sensor_file_synch_time))

        for i in GNSS_raw_data:
            raw_data = np.array([i[0] + time_delay, i[1], i[2], i[3], 0, 0, 0, 1])
            sensor_data.append(raw_data)
        sensor_data = np.array(sensor_data)
    if Sensor == "Robosense_32":
        sensor_data = read_icp_odom_file(path_sensor_file)
        sensor_data = np.array(sensor_data)

    # Read Extrinsic results for RTS
    # Tf = theodo_u.read_saved_tf(path+"list_tf/TF_list_static_cp.csv")
    Tf = read_saved_tf(path + "list_tf/TF_list_linear_dynamic_4dof.csv")
    Tf_1 = Tf[0]
    Tf_12 = Tf[1]
    Tf_13 = Tf[2]

    for param in parameters:
        print(param)

        if param[0] == 0:
            filtering = False
        if param[0] == 1:
            filtering = True
        thresold_d = param[1]  # tau_r [m/s]
        thresold_a = param[2]  # tau_a [deg/s]
        thresold_e = param[3]  # tau_e [deg/s]
        limit_time_interval = param[4]  # tau_s [s]
        size_interval = 2

        Mode = "L"  # Interpolation choice: 1. L -> Linear interpolation, 2. SGP -> Gaussian Process with Stheno library
        limit_search = limit_time_interval
        save = False

        save_index_1 = []
        save_index_2 = []
        save_index_3 = []

        for fname, opath in zip(file, output):
            if not filtering:
                path_out = opath + "raw_prediction/"
            else:
                path_out = opath + "filtered_prediction/"

            if filtering:
                t1, t2, t3, tp1, tp2, tp3, d1, d2, d3, a1, a2, a3, e1, e2, e3 = read_rosbag_theodolite_without_tf_raw_data_pre_filtered(
                    fname)
                index_1_f = thresold_raw_data(t1, d1, a1, e1, thresold_d, thresold_a * 3.1415926 / 180,
                                                       thresold_e * 3.1415926 / 180, limit_time_interval)
                index_2_f = thresold_raw_data(t2, d2, a2, e2, thresold_d, thresold_a * 3.1415926 / 180,
                                                       thresold_e * 3.1415926 / 180, limit_time_interval)
                index_3_f = thresold_raw_data(t3, d3, a3, e3, thresold_d, thresold_a * 3.1415926 / 180,
                                                       thresold_e * 3.1415926 / 180, limit_time_interval)
                t1 = t1[index_1_f]
                t2 = t2[index_2_f]
                t3 = t3[index_3_f]
                tp1 = tp1[index_1_f].T
                tp2 = tp2[index_2_f].T
                tp3 = tp3[index_3_f].T
                print(len(t1), len(t2), len(t3))
            else:
                t1, t2, t3, tp1, tp2, tp3, d1, d2, d3, a1, a2, a3, e1, e2, e3 = read_rosbag_theodolite_without_tf_raw_data(
                    fname)
                print(len(t1), len(t2), len(t3))

            # Put trajectories in same frame
            tp1 = Tf_1 @ tp1
            tp2 = Tf_12 @ tp2
            tp3 = Tf_13 @ tp3

            list_interval, list_time = split_time_interval_all_data(t1, t2, t3, limit_time_interval)
            list_trajectories_split = merge_interval(list_interval, list_time, t1, t2, t3, limit_search)

            Prediction_1 = []
            Prediction_2 = []
            Prediction_3 = []
            T_prediction = []
            Index_sensor = []

            for i in tqdm(list_trajectories_split):

                index_1 = np.array([i[0, 0], i[1, 0]])
                index_2 = np.array([i[0, 1], i[1, 1]])
                index_3 = np.array([i[0, 2], i[1, 2]])

                save_index_1.append(index_1)
                save_index_2.append(index_2)
                save_index_3.append(index_3)

                begin = np.max([t1[index_1[0]], t2[index_2[0]], t3[index_3[0]]])
                end = np.min([t1[index_1[1]], t2[index_2[1]], t3[index_3[1]]])

                if abs(end - begin) > size_interval and begin < end:

                    Number = 0
                    T_prediction_sensor = []
                    for value_sensor_data in sensor_data:
                        if end >= value_sensor_data[0] >= begin:
                            T_prediction_sensor.append(value_sensor_data[0])
                            Index_sensor.append(Number)
                        Number = Number + 1

                    # rate = 10  #Hz
                    # T_prediction_init = torch.from_numpy(np.arange(begin, end, 1/rate))
                    # print(T_prediction_sensor)
                    T_prediction_init = torch.from_numpy(np.array(T_prediction_sensor))

                    # Linear interpolation
                    if Mode == "L" or Mode == "All":
                        T1, X1, Y1, Z1, T2, X2, Y2, Z2, T3, X3, Y3, Z3 = data_training_L(t1, t2, t3,
                                                                                          tp1, tp2, tp3,
                                                                                          index_1,
                                                                                          index_2,
                                                                                          index_3)
                        mx1, my1, mz1, mx2, my2, mz2, mx3, my3, mz3 = linear_interpolation(T1, X1, Y1,
                                                                                            Z1, T2, X2,
                                                                                            Y2, Z2, T3,
                                                                                            X3, Y3, Z3)

                        for i in T_prediction_init.numpy():
                            T_prediction.append(i)
                            P1_L, P2_L, P3_L = linear_prediction(i, 0, mx1, my1, mz1, mx2, my2, mz2,
                                                                              mx3, my3, mz3)
                            Prediction_1.append(P1_L)
                            Prediction_2.append(P2_L)
                            Prediction_3.append(P3_L)

            print("Interpolation finished !")

            if save:

                if (Mode == "L" or Mode == "All"):
                    if (filtering):
                        trajectoire = "f-" + str(thresold_d) + "-" + str(thresold_a) + "-" + str(
                            thresold_e) + "-" + str(limit_time_interval) + "-" + str(size_interval) + "-" + "-L"
                    else:
                        trajectoire = "nf-" + str(limit_time_interval) + "-" + str(size_interval) + "-" + "-L"

                    if save:
                        Convert_raw_data_point_to_csv(T_prediction, Prediction_1,
                                                               path_out + trajectoire + "_1.csv")
                        Convert_raw_data_point_to_csv(T_prediction, Prediction_2,
                                                               path_out + trajectoire + "_2.csv")
                        Convert_raw_data_point_to_csv(T_prediction, Prediction_3,
                                                               path_out + trajectoire + "_3.csv")

            print("Saved !")

    # Trajectories predicted
    Time = np.array(T_prediction)
    P1_arr = np.array(Prediction_1)
    P2_arr = np.array(Prediction_2)
    P3_arr = np.array(Prediction_3)
    Index_sensor = np.array(Index_sensor)

    file_sensors = if_file_exist(path + "sensors_extrinsic_calibration/calibration_results.csv", '')
    extrinsic_calibration_results = read_extrinsic_calibration_results_file(file_sensors)
    error = []
    error = inter_prism_distance_error_mean(P1_arr, P2_arr, P3_arr, extrinsic_calibration_results[0:3])

    # Outliers for results
    P1_sort = []
    P2_sort = []
    P3_sort = []
    Time_sort = []
    Index_sensor_sort = []
    threshold_inter_prims_error = 10  # In mm, need to check how to choose it
    for i, j, k, l, m, n in zip(error, P1_arr, P2_arr, P3_arr, T_prediction, Index_sensor):
        if i < threshold_inter_prims_error:
            P1_sort.append(j)
            P2_sort.append(k)
            P3_sort.append(l)
            Time_sort.append(m)
            Index_sensor_sort.append(n)

    P_sensor = load_sensor_position(path, Sensor, Gps_reference_chosen)

    # Doing a minimization between these not moving points, and the 3D prism coordinates
    # Pose_GNSS is a list of each rigid transform founded
    list_sensor_time = []
    Pose_sensor = []
    Prism_corrected = []
    number = len(P1_sort)
    for i in range(0, number):
        Q = np.array([P1_sort[i], P2_sort[i], P3_sort[i]]).T
        Q = np.concatenate((Q, np.array([[1, 1, 1]])), axis=0)
        T = point_to_point_minimization(P_sensor, Q)
        Pose_sensor.append(T)
        prism_correct = T @ P_sensor
        Prism_corrected.append(prism_correct)
        list_sensor_time.append(Time_sort[i])
    Pose_sensor_arr = np.array(Pose_sensor)
    Prism_corrected_arr = np.array(Prism_corrected)

    if Sensor == "GNSS":
        grountruth_GP_gps_convert_for_eval(list_sensor_time, Pose_sensor_arr,
                            path + "ground_truth/gps_predicted_" + str(Gps_reference_chosen) + ".txt")
        grountruth_GNSS_sorted_convert_for_eval(Index_sensor_sort, path_sensor_file, time_delay,
                            path + "gps_data/gps_sorted_eval_" + str(Gps_reference_chosen) + ".txt")

    if Sensor == "Robosense_32":
        grountruth_convert_for_eval(list_sensor_time, Pose_sensor_arr, path + "ground_truth/icp_predicted.txt")
        grountruth_ICP_sorted_convert_for_eval(Index_sensor_sort, path_sensor_file, path + "ICP/icp_sorted_eval.txt")

    print("Pipeline done !")

def load_sensor_position(path, Sensor, Gps_reference_chosen):
    # Load sensor positions
    file_sensor_positions = if_file_exist(path + "sensors_extrinsic_calibration/sensor_positions.csv", '')
    sensor_positions_list = read_extrinsic_calibration_results_file(file_sensor_positions)

    sensors = []
    p1_position_rts = np.array(sensor_positions_list[0])
    p2_position_rts = np.array(sensor_positions_list[1])
    p3_position_rts = np.array(sensor_positions_list[2])
    if Sensor == "GNSS":
        sensors.append(np.array(sensor_positions_list[3]))
        sensors.append(np.array(sensor_positions_list[4]))
        sensors.append(np.array(sensor_positions_list[5]))

        p1_position_gnss = p1_position_rts - sensors[Gps_reference_chosen - 1]
        p1_position_gnss[3] = 1
        p2_position_gnss = p2_position_rts - sensors[Gps_reference_chosen - 1]
        p2_position_gnss[3] = 1
        p3_position_gnss = p3_position_rts - sensors[Gps_reference_chosen - 1]
        p3_position_gnss[3] = 1

        p_sensor = np.array([p1_position_gnss,
                             p2_position_gnss,
                             p3_position_gnss]).T

    if Sensor == "Robosense_32":
        sensors.append(np.array(sensor_positions_list[6]))
        sensors.append(np.array(sensor_positions_list[7]))
        sensors.append(np.array(sensor_positions_list[8]))
        sensors.append(np.array(sensor_positions_list[9]))
        ux = sensors[1] - sensors[0]
        uy = sensors[2] - sensors[0]
        uz = sensors[3] - sensors[0]

        t_lidar = np.array([[ux[0], uy[0], uz[0], sensors[0][0]],
                            [ux[1], uy[1], uz[1], sensors[0][1]],
                            [ux[2], uy[2], uz[2], sensors[0][2]],
                            [0, 0, 0, 1]])
        t_lidar_inv = np.linalg.inv(t_lidar)
        p1_position_lidar = t_lidar_inv @ p1_position_rts
        p2_position_lidar = t_lidar_inv @ p2_position_rts
        p3_position_lidar = t_lidar_inv @ p3_position_rts

        p_sensor = np.array([p1_position_lidar,
                             p2_position_lidar,
                             p3_position_lidar]).T

    return p_sensor

def range_noise(range_value, random_noise, num_samples):
    return range_value*(1+random_noise[2]*10**(-6)) + np.random.normal(random_noise[0], random_noise[1], num_samples)

def elevation_noise(true_value, random_noise_precision, random_noise_tilt, num_samples):
    return true_value + np.random.normal(random_noise_precision[0], random_noise_precision[1], num_samples) + np.random.normal(random_noise_tilt[0], random_noise_tilt[1], num_samples)

def azimuth_noise(true_value, elevation_value, random_noise_precision, random_noise_tilt, num_samples):
    return true_value + np.random.normal(random_noise_precision[0], random_noise_precision[1], num_samples) + np.random.normal(random_noise_tilt[0], random_noise_tilt[1], num_samples)/np.tan(elevation_value)

def edm_noise(measured_edm, lambda_edm, Measured_values, Nominal_values, noise_temp, noise_pressure, noise_humidity, num_samples):
    N_gr = 287.6155 + 4.88660/(lambda_edm)**2 + 0.06800/(lambda_edm)**4
    x = 7.5*(Measured_values[1]+np.random.normal(noise_temp[0], noise_temp[1], num_samples))/(237.3+Measured_values[1]+np.random.normal(noise_temp[0], noise_temp[1], num_samples))+0.7857
    e = (Measured_values[2]+np.random.normal(noise_humidity[0], noise_humidity[1], num_samples))*(10**x)/100
    N_l = (273.15/1013.25)*(N_gr*(Measured_values[0]+np.random.normal(noise_pressure[0], noise_pressure[1], num_samples))/(273.15+Measured_values[1]+np.random.normal(noise_temp[0], noise_pressure[1], num_samples)))-11.27*e/(273.15+Measured_values[1]+np.random.normal(noise_temp[0], noise_pressure[1], num_samples))
    N_o = (273.15/1013.25)*(N_gr*Nominal_values[0]/(273.15+Nominal_values[1]))-11.27*Nominal_values[2]/(273.15+Nominal_values[1])
    ppm = (N_o - N_l)/(1+N_l*10**(-6))
    return measured_edm*(1+ppm*10**(-6))

def get_cov_ellipsoid(cov, mu=np.zeros((3)), nstd=3):
    """
    Return the 3d points representing the covariance matrix
    cov centred at mu and scaled by the factor nstd.
    Plot on your favourite 3d axis.
    Example 1:  ax.plot_wireframe(X,Y,Z,alpha=0.1)
    Example 2:  ax.plot_surface(X,Y,Z,alpha=0.1)
    """
    assert cov.shape==(3,3)

    # Find and sort eigenvalues to correspond to the covariance matrix
    eigvals, eigvecs = np.linalg.eigh(cov)
    idx = np.sum(cov,axis=0).argsort()
    eigvals_temp = eigvals[idx]
    idx = eigvals_temp.argsort()
    eigvals = eigvals[idx]
    eigvecs = eigvecs[:,idx]

    # Set of all spherical angles to draw our ellipsoid
    n_points = 100
    theta = np.linspace(0, 2*np.pi, n_points)
    phi = np.linspace(0, np.pi, n_points)

    # Width, height and depth of ellipsoid
    rx, ry, rz = nstd * np.sqrt(eigvals)

    # Get the xyz points for plotting
    # Cartesian coordinates that correspond to the spherical angles:
    X = rx * np.outer(np.cos(theta), np.sin(phi))
    Y = ry * np.outer(np.sin(theta), np.sin(phi))
    Z = rz * np.outer(np.ones_like(theta), np.cos(phi))

    # Rotate ellipsoid for off axis alignment
    old_shape = X.shape
    # Flatten to vectorise rotation
    X,Y,Z = X.flatten(), Y.flatten(), Z.flatten()
    X,Y,Z = np.matmul(eigvecs, np.array([X,Y,Z]))
    X,Y,Z = X.reshape(old_shape), Y.reshape(old_shape), Z.reshape(old_shape)

    # Add in offsets for the mean
    X = X + mu[0]
    Y = Y + mu[1]
    Z = Z + mu[2]

    return X,Y,Z

def return_error_marker(trimble_1_gcp, trimble_2_gcp, trimble_3_gcp, T_1_grand, T_2_grand, T_3_grand):
    tp1 = T_1_grand@trimble_1_gcp
    tp2 = T_2_grand@trimble_2_gcp
    tp3 = T_3_grand@trimble_3_gcp
    error = []
    for i,j,k in zip(tp1[0:3],tp2[0:3],tp3[0:3]):
        dist_12 = np.linalg.norm(i-j)
        dist_13 = np.linalg.norm(i-k)
        dist_23 = np.linalg.norm(k-j)
        error.append(dist_12)
        error.append(dist_13)
        error.append(dist_23)
    return error

def return_good_tf(min_array, min_index):
    T_1_choose = min_array[min_index,3]
    T_2_choose = min_array[min_index,4]
    T_3_choose = min_array[min_index,5]
    return T_1_choose, T_2_choose, T_3_choose

def calculate_uncertainty_resection(min_array, min_index):
    T_1_choose = min_array[min_index,3]
    r = R.from_matrix(T_1_choose[0:3,0:3])
    angle = r.as_euler('xyz', degrees=False)
    V_1_choose = np.array([T_1_choose[0,3],T_1_choose[1,3],T_1_choose[2,3],angle[0],angle[1],angle[2]])
    T_2_choose = min_array[min_index,4]
    r = R.from_matrix(T_2_choose[0:3,0:3])
    angle = r.as_euler('xyz', degrees=False)
    V_2_choose = np.array([T_2_choose[0,3],T_2_choose[1,3],T_2_choose[2,3],angle[0],angle[1],angle[2]])
    T_3_choose = min_array[min_index,5]
    r = R.from_matrix(T_3_choose[0:3,0:3])
    angle = r.as_euler('xyz', degrees=False)
    V_3_choose = np.array([T_3_choose[0,3],T_3_choose[1,3],T_3_choose[2,3],angle[0],angle[1],angle[2]])

    value_1 = []
    value_2 = []
    value_3 = []
    for i in np.array(min_array[:,3]):
        r = R.from_matrix(i[0:3,0:3])
        angle = r.as_euler('xyz', degrees=False)
        value_1.append(np.array([i[0,3],i[1,3],i[2,3], angle[0], angle[1], angle[2]]))
    for i in np.array(min_array[:,4]):
        r = R.from_matrix(i[0:3,0:3])
        angle = r.as_euler('xyz', degrees=False)
        value_2.append(np.array([i[0,3],i[1,3],i[2,3], angle[0], angle[1], angle[2]]))
    for i in np.array(min_array[:,5]):
        r = R.from_matrix(i[0:3,0:3])
        angle = r.as_euler('xyz', degrees=False)
        value_3.append(np.array([i[0,3],i[1,3],i[2,3], angle[0], angle[1], angle[2]]))
    value_1 = np.array(value_1)
    value_2 = np.array(value_2)
    value_3 = np.array(value_3)

    std_1 = [np.std(value_1[:,0]-V_1_choose[0]), np.std(value_1[:,1]-V_1_choose[1]), np.std(value_1[:,2]-V_1_choose[2]),
             np.std(value_1[:,3]-V_1_choose[3]), np.std(value_1[:,4]-V_1_choose[4]), np.std(value_1[:,5]-V_1_choose[5])]
    std_2 = [np.std(value_2[:,0]-V_2_choose[0]), np.std(value_2[:,1]-V_2_choose[1]), np.std(value_2[:,2]-V_2_choose[2]),
             np.std(value_2[:,3]-V_2_choose[3]), np.std(value_2[:,4]-V_2_choose[4]), np.std(value_2[:,5]-V_2_choose[5])]
    std_3 = [np.std(value_3[:,0]-V_3_choose[0]), np.std(value_3[:,1]-V_3_choose[1]), np.std(value_3[:,2]-V_3_choose[2]),
             np.std(value_3[:,3]-V_3_choose[3]), np.std(value_3[:,4]-V_3_choose[4]), np.std(value_3[:,5]-V_3_choose[5])]

    return T_1_choose, T_2_choose, T_3_choose, std_1, std_2, std_3

def function_noise_resection(file_name, number_test):
    min_1 = []
    min_2 = []
    min_3 = []
    for i in range(0,number_test):
        trimble_1_gcp, trimble_2_gcp, trimble_3_gcp, T_1_grand, T_2_grand, T_3_grand = read_marker_file(file_name, 1, 0.8)
        error_1 = return_error_marker(trimble_1_gcp, trimble_2_gcp, trimble_3_gcp, T_1_grand, T_2_grand, T_3_grand)
        min_1.append([trimble_1_gcp,trimble_2_gcp,trimble_3_gcp,T_1_grand,T_2_grand,T_3_grand,error_1])

        trimble_1_gcp, trimble_2_gcp, trimble_3_gcp, T_1_grand, T_2_grand, T_3_grand = read_marker_file(file_name, 2, 0.8)
        error_2 = return_error_marker(trimble_1_gcp, trimble_2_gcp, trimble_3_gcp, T_1_grand, T_2_grand, T_3_grand)
        min_2.append([trimble_1_gcp,trimble_2_gcp,trimble_3_gcp,T_1_grand,T_2_grand,T_3_grand,error_2])

        trimble_1_gcp, trimble_2_gcp, trimble_3_gcp, T_1_grand, T_2_grand, T_3_grand = read_marker_file(file_name, 3, 0.8)
        error_3 = return_error_marker(trimble_1_gcp, trimble_2_gcp, trimble_3_gcp, T_1_grand, T_2_grand, T_3_grand)
        min_3.append([trimble_1_gcp,trimble_2_gcp,trimble_3_gcp,T_1_grand,T_2_grand,T_3_grand,error_3])
    min_1 = np.array(min_1)
    min_2 = np.array(min_2)
    min_3 = np.array(min_3)
    list_min_1 = np.array([np.median(i) for i in min_1[:,6]])
    min_index_1 = np.where(list_min_1 == np.min(list_min_1))[0][0]
    list_min_2 = np.array([np.median(i) for i in min_2[:,6]])
    min_index_2 = np.where(list_min_2 == np.min(list_min_2))[0][0]
    list_min_3 = np.array([np.median(i) for i in min_3[:,6]])
    min_index_3 = np.where(list_min_3 == np.min(list_min_3))[0][0]
    ts_chosen = np.where([np.min(list_min_1), np.min(list_min_2), np.min(list_min_3)] == np.min([np.min(list_min_1), np.min(list_min_2), np.min(list_min_3)]))[0][0]
    print("Min median error [m]: ", round(np.min([np.min(list_min_1), np.min(list_min_2), np.min(list_min_3)]),5))
    print("RTS number ", ts_chosen, " taken as main frame")
    if(ts_chosen==0):
        return calculate_uncertainty_resection(min_1, min_index_1)
    if(ts_chosen==1):
        return calculate_uncertainty_resection(min_2, min_index_2)
    if(ts_chosen==2):
        return calculate_uncertainty_resection(min_3, min_index_3)

def correct_tf(T_init, std_noise, num_samples):
    T_corrected_list = []
    r = R.from_matrix(T_init[0:3,0:3])
    angle = r.as_euler('xyz', degrees=False)
    for i in range(0, num_samples):
        angle_corrected = np.array([angle[0]+np.random.normal(0, std_noise[3], 1),
                                    angle[1]+np.random.normal(0, std_noise[4], 1),
                                    angle[2]+np.random.normal(0, std_noise[5], 1)])
        r = R.from_euler('xyz', angle_corrected.flatten(), degrees=False)
        R_corrected = r.as_matrix()
        T_corrected = np.identity(4)
        T_corrected[0:3,0:3]= R_corrected
        T_change = np.array([np.random.normal(0, std_noise[0], 1),np.random.normal(0, std_noise[1], 1),np.random.normal(0, std_noise[2], 1),0]).T
        T_corrected[:,3] = T_init[:,3] + T_change
        T_corrected_list.append(T_corrected)
    return T_corrected_list

def MC_raw_data(num_samples, range_value, random_noise_range, true_azimuth, true_elevation, random_noise_angle, random_noise_tilt):

    # ## Atmospheric corrections
    # measured_edm = 20
    # lambda_edm =  0.905  # In micro-meter
    # Nominal_values = [1013.25, 0, 0]    # Nominal values for the TS (Pressure [hPa], temperature [C], humidity [%])
    # Measured_values = [1020, 5, 15]     # Pressure [hPa], temperature [C], humidity [%]
    # noise_temp = [0, 0.1]
    # noise_pressure = [0, 10]
    # noise_humidity = [0, 2]
    # edm_range = theodo_g.edm_noise(measured_edm, lambda_edm, Measured_values, Nominal_values, noise_temp, noise_pressure, noise_humidity, num_samples)

    dist = range_noise(range_value, random_noise_range, num_samples)
    elevation = elevation_noise(true_elevation, random_noise_angle, random_noise_tilt, num_samples)
    azimuth = azimuth_noise(true_azimuth, elevation, random_noise_angle, random_noise_tilt, num_samples)

    points_simulated = []
    for i, j, k in zip(dist, azimuth, elevation):
        point = give_points(i, j, k, 2)
        points_simulated.append(point)
    points_simulated = np.array(points_simulated)

    cov_matrix_simulated = np.cov(points_simulated.T[0:3, :])
    mu_points_simulated = np.mean(points_simulated.T[0:3, :], axis=1)
    mu_raw_data = give_points(range_value, true_azimuth,true_elevation, 2)

    return mu_raw_data, mu_points_simulated, cov_matrix_simulated

def list_static_points(trimble_used, limit_speed, limit_number):

    P_nm_list = find_not_moving_points(trimble_used, limit_speed)

    Groupe_index_list = []
    start = P_nm_list[0]
    list = []
    for i in P_nm_list:
        if np.linalg.norm(trimble_used.T[start][0:3] - trimble_used.T[i][0:3]) < limit_speed:
            list.append(i)
        else:
            if (len(list) > limit_number):
                Groupe_index_list.append(list)
            list = []
            start = i
    if len(list) > 0:
        Groupe_index_list.append(list)
    return Groupe_index_list

def unit_return_covariance_point(trimble_used, dist_used, azimuth_used, elevation_used, Groupe_index_1_list, list_number):
    P_1_0 = trimble_used.T[Groupe_index_1_list[list_number]]
    D_1_0 = dist_used.T[Groupe_index_1_list[list_number]]
    A_1_0 = azimuth_used.T[Groupe_index_1_list[list_number]]
    E_1_0 = elevation_used.T[Groupe_index_1_list[list_number]]
    #Cov_1_0 = np.cov(P_1_0.T[0:3])
    cov_matrix = np.cov(P_1_0.T[0:3, :])
    mu_points = np.mean(P_1_0.T[0:3, :], axis=1)
    mean_D_1_0 = np.mean(D_1_0)
    mean_A_1_0 = np.mean(A_1_0)
    mean_E_1_0 = np.mean(E_1_0)
    return mu_points, cov_matrix, mean_D_1_0, mean_A_1_0, mean_E_1_0, P_1_0

def return_uncertainty_of_discrete_trajectory(trimble_used, dist_used, azimuth_used, elevation_used, Groupe_index_list):
    trajectory_list = []
    for i in range(0, len(Groupe_index_list)):
        # Good data
        D_1_0 = dist_used.T[Groupe_index_list[i]]
        A_1_0 = azimuth_used.T[Groupe_index_list[i]]
        E_1_0 = elevation_used.T[Groupe_index_list[i]]

        # MC
        num_samples = 1000
        ## Range
        range_value = np.mean(D_1_0)
        random_noise_range = [0, 0.004 / 2, 2]  ## Mean, sigma, ppm,  4mm + 2ppm (2 sigma)  ISO17123-3
        ## Angles
        true_azimuth = np.mean(A_1_0)
        true_elevation = np.mean(E_1_0)
        random_noise_angle = [0,
                              0.000024241 / 5 * 4 / 2]  # Mean, sigma, 5"=0.000024241 précison datasheet  (2 sigma)  ISO17123-3
        ## Tilt compensator
        random_noise_tilt = [0,
                             0.000002424 / 2]  # Mean, sigma, 0.5"=0.000002424 précison datasheet  (2 sigma)  ISO17123-3
        mu_raw_data, mu_points_simulated, cov_matrix_simulated = MC_raw_data(num_samples, range_value,
                                                                              random_noise_range, true_azimuth,
                                                                              true_elevation,
                                                                              random_noise_angle,
                                                                              random_noise_tilt)

        trajectory_list.append([mu_raw_data, cov_matrix_simulated])
    return np.array(trajectory_list)

def return_point_from_covariance(cov, p, num_samples):
    eigvals, eigvecs = np.linalg.eigh(cov)
    idx = np.sum(cov, axis=0).argsort()
    eigvals_temp = eigvals[idx]
    idx = eigvals_temp.argsort()
    eigvals = eigvals[idx]
    eigvecs = eigvecs[:, idx]
    sigma_values = np.sqrt(eigvals)

    correction_1 = np.random.normal(0, sigma_values[0], num_samples)
    correction_2 = np.random.normal(0, sigma_values[1], num_samples)
    correction_3 = np.random.normal(0, sigma_values[2], num_samples)

    p_corrected = []
    for i,j,k in zip(correction_1,correction_2,correction_3):
        p_corrected.append(p[0:3] + i*eigvecs[0] + j*eigvecs[1] + k*eigvecs[2])
    return p_corrected

def find_noise_list_tf(T_list):
    T_x = []
    T_y = []
    T_z = []
    T_roll = []
    T_pitch = []
    T_yaw = []
    for i in T_list:
        r = R.from_matrix(i[0:3, 0:3])
        angle = r.as_euler('xyz', degrees=False)
        T_x.append(i[0,3])
        T_y.append(i[1,3])
        T_z.append(i[2, 3])
        T_roll.append(angle[0])
        T_pitch.append(angle[1])
        T_yaw.append(angle[2])
    mu_x = np.mean(T_x)
    mu_y = np.mean(T_y)
    mu_z = np.mean(T_z)
    # mu_roll = np.mean(T_roll)
    # mu_pitch = np.mean(T_pitch)
    # mu_yaw = np.mean(T_yaw)
    # std_x = np.std(T_x)
    # std_y = np.std(T_y)
    # std_z = np.std(T_z)
    # std_roll = np.std(T_roll)
    # std_pitch = np.std(T_pitch)
    # std_yaw = np.std(T_yaw)
    p_T = np.array([T_x, T_y, T_z])
    # print(np.array(p_T))
    cov_matrix = np.cov(p_T)
    mu_points = np.mean(p_T, axis=1)
    # print(cov_matrix)
    # print(mu_points)
    return p_T, mu_points, cov_matrix

def STEAM_interpolation_with_covariance(Time_RTS, Time_sensor, MC_data):
    ## STEAM 1
    num_states = len(Time_RTS)  # total number of states
    fake_meas = np.empty((num_states, 4, 1))
    iterator_meas = 0
    for mc1 in MC_data:
        fake_meas[iterator_meas, 0, 0] = mc1[1][0]
        fake_meas[iterator_meas, 1, 0] = mc1[1][1]
        fake_meas[iterator_meas, 2, 0] = mc1[1][2]
        fake_meas[iterator_meas, 3, 0] = 1
        iterator_meas += 1
    # states with initial conditions and associated timestamps
    # T_ba = T_k0 where 0 can be some global frame (e.g. UTM) and k is the vehicle/robot frame at time k
    states = [(Time_RTS[i], Transformation(C_ba=np.eye(3), r_ba_in_a=fake_meas[i, :3]), np.zeros((6, 1))) for i in
              range(num_states)]
    # wrap states with corresponding steam state variables (no copying!)
    state_vars = [(t, SE3StateVar(T_vi), VSpaceStateVar(w_iv_inv)) for t, T_vi, w_iv_inv in states]
    qcd = np.ones(6)
    traj = TrajectoryInterface(qcd=qcd)
    for t, T_vi, w_iv_inv in state_vars:
        traj.add_knot(time=Time(t), T_k0=T_vi, w_0k_ink=w_iv_inv)
    cost_terms = []
    # use a shared L2 loss function and noise model for all cost terms
    loss_func = L2LossFunc()
    # noise_model = StaticNoiseModel(3*np.eye(3), "information")
    # noise_model = DynamicNoiseModel()
    for i, j in zip(range(num_states), MC_data):
        noise_model = StaticNoiseModel(np.linalg.inv(j[2]), "information")
        error_func = P2PErrorEvaluator(T_rq=state_vars[i][1], reference=np.array([[0, 0, 0, 1]]).T, query=fake_meas[i])
        cost_terms.append(WeightedLeastSquareCostTerm(error_func, noise_model, loss_func))

    Error_STEAM = False
    try:
        opt_prob = OptimizationProblem()
        opt_prob.add_state_var(*[v for state_var in state_vars for v in state_var[1:]])
        opt_prob.add_cost_term(*traj.get_prior_cost_terms())
        opt_prob.add_cost_term(*cost_terms)
        solver = GaussNewtonSolver(opt_prob, verbose=False)
        solver.optimize()
    except:
        print("Exception interpolation !")
        Error_STEAM = True

    if Error_STEAM==False:
        time_interpolated = np.array(Time_sensor)
        Error_STEAM = False
        try:
            T_k0_interp0 = [
                traj.get_pose_interpolator(Time(i)).evaluate().matrix() for i in time_interpolated
            ]
            T_0k_interp = np.array([np.linalg.inv(x) for x in T_k0_interp0])
            covariance = Covariance(opt_prob)
            T_k0_interp1 = [
                traj.get_covariance(covariance, Time(i)) for i in time_interpolated
            ]
            MC_interpolated = []
            for in0, in1, in2 in zip(time_interpolated, T_0k_interp, T_k0_interp1):
                MC_interpolated.append([in0, in1[0:3, 3], in2[0:3, 0:3]])
        except:
            print("Exception extraction !")
            Error_STEAM = True

        if Error_STEAM == False:
            return MC_interpolated
        else:
            return []

    else:
        return []



def plot_frame(ax, T_ri=np.eye(4), length=1, **kwargs):
  axes = T_ri @ np.concatenate((np.eye(3) * length, np.ones((1, 3))), axis=0)
  ax.plot(*zip(T_ri[:3, 3], axes[:3, 0]), color='r', **kwargs)
  ax.plot(*zip(T_ri[:3, 3], axes[:3, 1]), color='g', **kwargs)
  ax.plot(*zip(T_ri[:3, 3], axes[:3, 2]), color='b', **kwargs)

def get_cov_ellipsoid(ax, mu, cov, nstd=3, **kwargs):
  assert mu.shape == (3,) and cov.shape == (3, 3)

  # Find and sort eigenvalues to correspond to the covariance matrix
  eigvals, eigvecs = np.linalg.eigh(cov)
  idx = np.sum(cov, axis=0).argsort()
  eigvals_temp = eigvals[idx]
  idx = eigvals_temp.argsort()
  eigvals = eigvals[idx]
  eigvecs = eigvecs[:, idx]

  # Set of all spherical angles to draw our ellipsoid
  n_points = 100
  theta = np.linspace(0, 2 * np.pi, n_points)
  phi = np.linspace(0, np.pi, n_points)

  # Width, height and depth of ellipsoid
  rx, ry, rz = nstd * np.sqrt(eigvals)

  # Get the xyz points for plotting
  # Cartesian coordinates that correspond to the spherical angles:
  X = rx * np.outer(np.cos(theta), np.sin(phi))
  Y = ry * np.outer(np.sin(theta), np.sin(phi))
  Z = rz * np.outer(np.ones_like(theta), np.cos(phi))

  # Rotate ellipsoid for off axis alignment
  old_shape = X.shape
  # Flatten to vectorise rotation
  X, Y, Z = X.flatten(), Y.flatten(), Z.flatten()
  X, Y, Z = np.matmul(eigvecs, np.array([X, Y, Z]))
  X, Y, Z = X.reshape(old_shape), Y.reshape(old_shape), Z.reshape(old_shape)

  # Add in offsets for the mean
  X = X + mu[0]
  Y = Y + mu[1]
  Z = Z + mu[2]

  return ax.plot_wireframe(X, Y, Z, **kwargs)

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def chose_sensor_before_ptp(path, Sensor, Gps_reference_chosen):
    # Load sensor positions
    file_sensor_positions = if_file_exist(path + "sensors_extrinsic_calibration/sensor_positions.csv", '')
    sensor_positions_list = read_extrinsic_calibration_results_file(file_sensor_positions)

    Sensors = []
    P1_position_RTS = np.array(sensor_positions_list[0])
    P2_position_RTS = np.array(sensor_positions_list[1])
    P3_position_RTS = np.array(sensor_positions_list[2])
    if Sensor == "GNSS":
        Sensors.append(np.array(sensor_positions_list[3]))
        Sensors.append(np.array(sensor_positions_list[4]))
        Sensors.append(np.array(sensor_positions_list[5]))

        P1_position_GNSS = P1_position_RTS - Sensors[Gps_reference_chosen - 1]
        P1_position_GNSS[3] = 1
        P2_position_GNSS = P2_position_RTS - Sensors[Gps_reference_chosen - 1]
        P2_position_GNSS[3] = 1
        P3_position_GNSS = P3_position_RTS - Sensors[Gps_reference_chosen - 1]
        P3_position_GNSS[3] = 1

        P_sensor = np.array([P1_position_GNSS,
                             P2_position_GNSS,
                             P3_position_GNSS]).T

        return P_sensor

    if Sensor == "Robosense_32":
        Sensors.append(np.array(sensor_positions_list[6]))
        Sensors.append(np.array(sensor_positions_list[7]))
        Sensors.append(np.array(sensor_positions_list[8]))
        Sensors.append(np.array(sensor_positions_list[9]))
        ux = Sensors[1] - Sensors[0]
        uy = Sensors[2] - Sensors[0]
        uz = Sensors[3] - Sensors[0]

        T_lidar = np.array([[ux[0], uy[0], uz[0], Sensors[0][0]],
                            [ux[1], uy[1], uz[1], Sensors[0][1]],
                            [ux[2], uy[2], uz[2], Sensors[0][2]],
                            [0, 0, 0, 1]])
        T_lidar_inv = np.linalg.inv(T_lidar)
        P1_position_lidar = T_lidar_inv @ P1_position_RTS
        P2_position_lidar = T_lidar_inv @ P2_position_RTS
        P3_position_lidar = T_lidar_inv @ P3_position_RTS

        P_sensor = np.array([P1_position_lidar,
                             P2_position_lidar,
                             P3_position_lidar]).T

        return P_sensor