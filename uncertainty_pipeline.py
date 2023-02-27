import importlib
import random
import time

import numpy as np
import torch

import scripts.groundtruth_utils as theodo_g
import scripts.prediction_utils as predict_u
import scripts.theodolite_function as theodo_f
import scripts.theodolite_plot_function as theodo_p
import scripts.theodolite_utils as theodo_u
from uncertainty_consts import *

theodo_u = importlib.reload(theodo_u)
theodo_p = importlib.reload(theodo_p)
theodo_f = importlib.reload(theodo_f)
predict_u = importlib.reload(predict_u)

# SEEDS
np.random.seed(10)
random.seed(10)

for file_i, weather_index in zip(paths, weather_list):
    print("File processed: ", file_i)
    # Reading sensor extrinsic calibration
    file_sensors = theodo_u.if_file_exist(
        file_i + "sensors_extrinsic_calibration/calibration_results.csv", ""
    )
    extrinsic_calibration_results = theodo_u.read_extrinsic_calibration_results_file(
        file_sensors
    )

    ## Read sensor data which we want a ground truth
    sensor_data = []

    ## Number of rigid transforms to apply the uncertainty
    path_file_GCP = "total_stations/GCP.txt"
    (
        frame_chosen,
        T_1_corrected,
        T_2_corrected,
        T_3_corrected,
    ) = theodo_g.extrinsic_calibration_noise(
        file_i + path_file_GCP,
        random_noise_range,
        random_noise_angle,
        random_noise_tilt,
        num_samples,
    )
    ## Read Extrinsic results for RTS
    ## Estimated extrinsic calibration uncertainty
    _, _, _, Tf_1, Tf_2, Tf_3 = theodo_u.read_marker_file(file_i + path_file_GCP, 1, 1)

    if weather_index == 0:
        data_weather = data_weather_quebec
    else:
        if weather_index == 1:
            data_weather = data_weather_fm

    for param in parameters:
        print("Parameters used: ", param)

        ## Parameters to process the rosbag data
        if param[0] == 0:
            filtering = False
        if param[0] == 1:
            filtering = True
        thresold_d = param[1]  ## tau_r [m/s]
        thresold_a = param[2]  ## tau_a [deg/s]
        thresold_e = param[3]  ## tau_e [deg/s]
        limit_time_interval = param[4]  ## tau_s [s]
        limit_search = limit_time_interval
        size_interval = param[5]  ## tau_l
        Mode = "STEAM"  ## Interpolation choice: 1. L -> Linear interpolation, 2. SGP -> Gaussian Process with Stheno library, 3. STEAM

        if type_file_input == "CSV":
            sub_path = "uncertainty/raw_data/"
            P1 = theodo_u.read_raw_data_uncertainty_speed(
                file_i + sub_path + "speed_prism1.csv"
            )
            P2 = theodo_u.read_raw_data_uncertainty_speed(
                file_i + sub_path + "speed_prism2.csv"
            )
            P3 = theodo_u.read_raw_data_uncertainty_speed(
                file_i + sub_path + "speed_prism3.csv"
            )
            (
                t1,
                t2,
                t3,
                d1,
                d2,
                d3,
                a1,
                a2,
                a3,
                e1,
                e2,
                e3,
                s1,
                s2,
                s3,
                ss1,
                ss2,
                ss3,
            ) = ([] for i in range(18))
            for p1_i, p2_i, p3_i in zip(P1, P2, P3):
                t1.append(p1_i[0])
                t2.append(p2_i[0])
                t3.append(p3_i[0])
                d1.append(p1_i[1])
                d2.append(p2_i[1])
                d3.append(p3_i[1])
                a1.append(p1_i[2])
                a2.append(p2_i[2])
                a3.append(p3_i[2])
                e1.append(p1_i[3])
                e2.append(p2_i[3])
                e3.append(p3_i[3])
                s1.append(p1_i[4])
                s2.append(p2_i[4])
                s3.append(p3_i[4])
                ss1.append(p1_i[5])
                ss2.append(p2_i[5])
                ss3.append(p3_i[5])
            t1 = np.array(t1)
            t2 = np.array(t2)
            t3 = np.array(t3)
            d1 = np.array(d1)
            d2 = np.array(d2)
            d3 = np.array(d3)
            a1 = np.array(a1)
            a2 = np.array(a2)
            a3 = np.array(a3)
            e1 = np.array(e1)
            e2 = np.array(e2)
            e3 = np.array(e3)
            s1 = np.array(s1)
            s2 = np.array(s2)
            s3 = np.array(s3)
            ss1 = np.array(ss1)
            ss2 = np.array(ss2)
            ss3 = np.array(ss3)

        ## Split trajectories according to tau_s
        list_interval, list_time = theodo_f.split_time_interval_all_data(
            t1, t2, t3, limit_time_interval
        )
        list_trajectories_split = theodo_f.merge_interval(
            list_interval, list_time, t1, t2, t3, limit_search
        )

        M_1_before_inter = []
        M_2_before_inter = []
        M_3_before_inter = []
        Prediction_1 = []
        Prediction_2 = []
        Prediction_3 = []
        T_prediction = []
        Index_sensor = []

        print("Number of sub-trajectories :", len(list_trajectories_split))
        number_ite = 0
        ## Process each of the sub-trajectories
        for i in list_trajectories_split:
            print("Sub-trajectories " + str(number_ite))
            number_ite = number_ite + 1
            index_1 = np.array([i[0, 0], i[1, 0]])
            index_2 = np.array([i[0, 1], i[1, 1]])
            index_3 = np.array([i[0, 2], i[1, 2]])

            begin = np.max([t1[index_1[0]], t2[index_2[0]], t3[index_3[0]]])
            end = np.min([t1[index_1[1]], t2[index_2[1]], t3[index_3[1]]])

            if (
                abs(end - begin) > size_interval and begin < end
            ):  # control number of sample
                ## If fake querrying, apply desired rate
                T_prediction_sensor = []
                if Sensor != "Fake":
                    Number = 0
                    for value_sensor_data in sensor_data:
                        if end >= value_sensor_data[0] >= begin:
                            T_prediction_sensor.append(value_sensor_data[0])
                            Index_sensor.append(Number)
                        Number = Number + 1

                    T_prediction_init = torch.from_numpy(np.array(T_prediction_sensor))
                else:
                    List_time = np.arange(begin, end, 1 / rate_fake)
                    for i in List_time:
                        T_prediction_sensor.append(i)
                    T_prediction_init = torch.from_numpy(
                        np.arange(begin, end, 1 / rate_fake)
                    )

                if Mode == "STEAM" or Mode == "All":
                    MC_1 = []
                    MC_2 = []
                    MC_3 = []
                    tic = time.perf_counter()
                    ## Compute uncertainty
                    T1, D1, A1, E1, S1, SS1 = predict_u.data_training_L_Raw_data(
                        t1, d1, a1, e1, s1, ss1, index_1
                    )
                    T2, D2, A2, E2, S2, SS2 = predict_u.data_training_L_Raw_data(
                        t2, d2, a2, e2, s2, ss2, index_2
                    )
                    T3, D3, A3, E3, S3, SS3 = predict_u.data_training_L_Raw_data(
                        t3, d3, a3, e3, s3, ss3, index_3
                    )
                    print("Number of input: ", len(T1))

                    for i1, j1, k1, l1, m1, n1 in zip(T1, D1, A1, E1, S1, SS1):
                        mu_raw_data, _, cov_matrix_simulated = theodo_g.MC_raw_data(
                            num_samples,
                            j1,
                            random_noise_range,
                            k1,
                            l1,
                            random_noise_angle,
                            random_noise_tilt,
                            Tf_1,
                            T_1_corrected,
                            data_weather,
                            i1,
                            m1,
                            n1,
                            time_error_synch_mean,
                            time_error_synch_std,
                            model_chosen,
                        )
                        MC_1.append([i1, mu_raw_data, cov_matrix_simulated])
                        M_1_before_inter.append([i1, mu_raw_data, cov_matrix_simulated])
                    for i2, j2, k2, l2, m2, n2 in zip(T2, D2, A2, E2, S2, SS2):
                        mu_raw_data, _, cov_matrix_simulated = theodo_g.MC_raw_data(
                            num_samples,
                            j2,
                            random_noise_range,
                            k2,
                            l2,
                            random_noise_angle,
                            random_noise_tilt,
                            Tf_2,
                            T_2_corrected,
                            data_weather,
                            i2,
                            m2,
                            n2,
                            time_error_synch_mean,
                            time_error_synch_std,
                            model_chosen,
                        )
                        MC_2.append([i2, mu_raw_data, cov_matrix_simulated])
                        M_2_before_inter.append([i2, mu_raw_data, cov_matrix_simulated])
                    for i3, j3, k3, l3, m3, n3 in zip(T3, D3, A3, E3, S3, SS3):
                        mu_raw_data, _, cov_matrix_simulated = theodo_g.MC_raw_data(
                            num_samples,
                            j3,
                            random_noise_range,
                            k3,
                            l3,
                            random_noise_angle,
                            random_noise_tilt,
                            Tf_3,
                            T_3_corrected,
                            data_weather,
                            i3,
                            m3,
                            n3,
                            time_error_synch_mean,
                            time_error_synch_std,
                            model_chosen,
                        )
                        MC_3.append([i3, mu_raw_data, cov_matrix_simulated])
                        M_3_before_inter.append([i3, mu_raw_data, cov_matrix_simulated])

                    ## STEAM
                    MC_1_interpolated = theodo_g.STEAM_interpolation_with_covariance(
                        T1, T_prediction_sensor, MC_1
                    )
                    MC_2_interpolated = theodo_g.STEAM_interpolation_with_covariance(
                        T2, T_prediction_sensor, MC_2
                    )
                    MC_3_interpolated = theodo_g.STEAM_interpolation_with_covariance(
                        T3, T_prediction_sensor, MC_3
                    )
                    toc = time.perf_counter()
                    print("Time [s]: ", round(toc - tic, 2))
                    if (
                        len(MC_1_interpolated) > 0
                        and len(MC_2_interpolated) > 0
                        and len(MC_3_interpolated) > 0
                    ):
                        Prediction_1.append(MC_1_interpolated)
                        Prediction_2.append(MC_2_interpolated)
                        Prediction_3.append(MC_3_interpolated)
                        T_prediction.append(T_prediction_sensor)

        MC_1_inter = []
        MC_2_inter = []
        MC_3_inter = []
        for i1, j1, k1 in zip(Prediction_1, Prediction_2, Prediction_3):
            for i2, j2, k2 in zip(i1, j1, k1):
                # Check on uncertainty. If too high, remove triplet
                if limit_taken_into_accout:
                    eig1 = np.linalg.eigvals(i2[2]) ** 0.5
                    eig2 = np.linalg.eigvals(j2[2]) ** 0.5
                    eig3 = np.linalg.eigvals(k2[2]) ** 0.5
                    if (
                        eig1[0] < limit_uncertainty
                        and eig1[1] < limit_uncertainty
                        and eig1[2] < limit_uncertainty
                        and eig2[0] < limit_uncertainty
                        and eig2[1] < limit_uncertainty
                        and eig2[2] < limit_uncertainty
                        and eig3[0] < limit_uncertainty
                        and eig3[1] < limit_uncertainty
                        and eig3[2] < limit_uncertainty
                    ):
                        MC_1_inter.append(i2)
                        MC_2_inter.append(j2)
                        MC_3_inter.append(k2)
                else:
                    MC_1_inter.append(i2)
                    MC_2_inter.append(j2)
                    MC_3_inter.append(k2)

        if save_MC_inteprolated:
            theodo_u.save_MC_interpolated_sorted(
                MC_1_inter,
                file_i
                + "uncertainty/interpolation/MC_"
                + str(Sensor)
                + "_"
                + str(model_chosen[0])
                + "_"
                + str(model_chosen[1])
                + "_"
                + str(model_chosen[2])
                + "_"
                + str(model_chosen[3])
                + "_"
                + str(model_chosen[4])
                + "_"
                + str(rate_fake)
                + "_1.csv",
            )
            theodo_u.save_MC_interpolated_sorted(
                MC_2_inter,
                file_i
                + "uncertainty/interpolation/MC_"
                + str(Sensor)
                + "_"
                + "_".join([*map(str, model_chosen)])
                + "_"
                + str(rate_fake)
                + "_2.csv",
            )
            theodo_u.save_MC_interpolated_sorted(
                MC_3_inter,
                file_i
                + "uncertainty/interpolation/MC_"
                + str(Sensor)
                + "_"
                + "_".join([*map(str, model_chosen)])
                + "_"
                + str(rate_fake)
                + "_3.csv",
            )

        MC_1_sorted = []
        MC_2_sorted = []
        MC_3_sorted = []
        if limit_taken_into_accout:
            for i1, j1, k1 in zip(MC_1_inter, MC_2_inter, MC_3_inter):
                d1 = abs(
                    np.linalg.norm(i1[1] - j1[1]) - extrinsic_calibration_results[0]
                )
                d2 = abs(
                    np.linalg.norm(i1[1] - k1[1]) - extrinsic_calibration_results[1]
                )
                d3 = abs(
                    np.linalg.norm(k1[1] - j1[1]) - extrinsic_calibration_results[2]
                )
                if d1 < limit_dist and d2 < limit_dist and d3 < limit_dist:
                    MC_1_sorted.append(i1)
                    MC_2_sorted.append(j1)
                    MC_3_sorted.append(k1)
        else:
            MC_1_sorted = MC_1_inter
            MC_2_sorted = MC_2_inter
            MC_3_sorted = MC_3_inter

        if want_sensor_pose:
            if Sensor != "Fake":
                P_sensor = theodo_g.chose_sensor_before_ptp(
                    path, Sensor, Gps_reference_chosen
                )
                Pose_sensor_MC = []
                for i_mc, j_mc, k_mc in zip(MC_1_sorted, MC_2_sorted, MC_3_sorted):
                    Pose_sensor = []
                    p1_corrected = theodo_g.return_point_from_covariance(
                        i_mc[2], i_mc[1], num_samples_MC_sensor
                    )
                    p2_corrected = theodo_g.return_point_from_covariance(
                        j_mc[2], j_mc[1], num_samples_MC_sensor
                    )
                    p3_corrected = theodo_g.return_point_from_covariance(
                        k_mc[2], k_mc[1], num_samples_MC_sensor
                    )
                    for i, j, k in zip(p1_corrected, p2_corrected, p3_corrected):
                        Q = np.array([i, j, k]).T
                        Q = np.concatenate((Q, np.array([[1, 1, 1]])), axis=0)
                        T = theodo_u.point_to_point_minimization(P_sensor, Q)
                        Pose_sensor.append(T)
                    p_T, mu_T, cov_T = theodo_g.find_noise_list_tf(Pose_sensor)
                    Pose_sensor_MC.append([i_mc[0], mu_T, cov_T])
            else:
                P_sensor = theodo_g.chose_sensor_before_ptp(
                    path, Sensor_fake, Gps_reference_chosen
                )
                Pose_sensor_MC = []
                for i_mc, j_mc, k_mc in zip(MC_1_sorted, MC_2_sorted, MC_3_sorted):
                    Pose_sensor = []
                    p1_corrected = theodo_g.return_point_from_covariance(
                        i_mc[2], i_mc[1], num_samples_MC_sensor
                    )
                    p2_corrected = theodo_g.return_point_from_covariance(
                        j_mc[2], j_mc[1], num_samples_MC_sensor
                    )
                    p3_corrected = theodo_g.return_point_from_covariance(
                        k_mc[2], k_mc[1], num_samples_MC_sensor
                    )
                    for i, j, k in zip(p1_corrected, p2_corrected, p3_corrected):
                        Q = np.array([i, j, k]).T
                        Q = np.concatenate((Q, np.array([[1, 1, 1]])), axis=0)
                        T = theodo_u.point_to_point_minimization(P_sensor, Q)
                        Pose_sensor.append(T)
                    p_T, mu_T, cov_T = theodo_g.find_noise_list_tf(Pose_sensor)
                    Pose_sensor_MC.append([i_mc[0], mu_T, cov_T])

        print("Interpolation finished !")
