import numpy as np
from scripts.theodolite_utils import *
from scripts.theodolite_function import *
from scripts.prediction_utils import *
from scripts.resection_functions import *

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