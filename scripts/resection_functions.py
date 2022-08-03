import numpy as np
from scripts import theodolite_utils as tu
import scipy.linalg
import scipy.optimize
import time
import scripts.liblie as ll
from scipy.spatial.transform import Rotation as R

def R_z(theta):
    R = np.eye(4, 4)
    R[0, :2] = [np.cos(theta), -np.sin(theta)]
    R[1, :2] = [np.sin(theta), np.cos(theta)]
    return R

def T_z(theta, t):
    T = R_z(theta)
    T[:3, 3] = t
    return T

def dist(p, q):
    return np.linalg.norm(p[0:3] - q[0:3])

def T_z_so3(x, y, z, theta):
    T = np.eye(4, 4)
    T[:3, 3] = np.array([x, y, z])
    C = ll.expm_so3(ll.wedge_so3(np.array([0, 0, theta])))
    T[:3, :3] = C
    return T

def cost_fun_one_pose(X, ps1, ps2):
    T = T_z_so3(*X)
    c = 0
    for i in range(ps1.shape[1]):
        p1 = ps1[:, i]
        p2 = ps2[:, i]
        c += np.linalg.norm(p1 - T@p2)**2
    return c/ps1.shape[1]

def cost_fun(p1s_l, p2s_l, p3s_l, xi_12, xi_13, d_truth12, d_truth13, d_truth23):
    T12 = exp_T(xi_12)
    T13 = exp_T(xi_13)
    c = 0
    c += sum([(dist(p1, T12@p2) - d_truth12)**2 for p1, p2 in zip(p1s_l, p2s_l)])
    c += sum([(dist(p1, T13@p3) - d_truth13)**2 for p1, p3 in zip(p1s_l, p3s_l)])
    c += sum([(dist(T12@p2, T13@p3) - d_truth23)**2 for p2, p3 in zip(p2s_l, p3s_l)])

    return c

def cost_fun_ls(p1s_l, p2s_l, p3s_l, xi_12, xi_13, d_truth12, d_truth13, d_truth23):
    T12 = exp_T(xi_12)
    T13 = exp_T(xi_13)

    N = len(p1s_l)
    c = np.zeros((N*3))
    c[:N] = [(dist(p1, T12@p2) - d_truth12) for p1, p2 in zip(p1s_l, p2s_l)]
    c[N:2*N] = [(dist(p1, T13@p3) - d_truth13) for p1, p3 in zip(p1s_l, p3s_l)]
    c[2*N:] = [(dist(T12@p2, T13@p3) - d_truth23) for p2, p3 in zip(p2s_l, p3s_l)]
    return c/(3*N)

def cost_fun_ls_4dof(p1s_l, p2s_l, p3s_l, xi_12, xi_13, d_truth12, d_truth13, d_truth23):
    T12 = exp_T_4dof(xi_12)
    T13 = exp_T_4dof(xi_13)

    N = len(p1s_l)
    c = np.zeros((N*3))
    c[:N] = [(dist(p1, T12@p2) - d_truth12) for p1, p2 in zip(p1s_l, p2s_l)]
    c[N:2*N] = [(dist(p1, T13@p3) - d_truth13) for p1, p3 in zip(p1s_l, p3s_l)]
    c[2*N:] = [(dist(T12@p2, T13@p3) - d_truth23) for p2, p3 in zip(p2s_l, p3s_l)]
    return c/(3*N)

def vee(xi):  ## TODO: use LibLie wedge
    T = np.zeros((4, 4))
    T[:3, :3] = np.array([[0, -xi[2], xi[1]],
                          [xi[2], 0, -xi[0]],
                          [-xi[1], xi[0], 0]])
    T[:3, 3] = xi[3:]
    return T

def vee_4dof(xi):  ## TODO: use LibLie wedge
    T = np.zeros((4, 4))
    T[:3, :3] = np.array([[0, -xi[0], 0],
                          [xi[0], 0, 0],
                          [0, 0, 0]])
    T[:3, 3] = xi[1:]
    return T

def exp_T_4dof(xi):  ## TODO: use LibLie
    return scipy.linalg.expm(vee_4dof(xi))

def exp_T(xi):  ## TODO: use LibLie
    return scipy.linalg.expm(vee(xi))

def exp_inv_T(A):  ## TODO: use LibLie
    return scipy.linalg.logm(A)

def dynamic_vs_static_control_points_error_comparison(static_file_path: str, dynamic_file_path: str,
                                                      training_threshold: float = 0.75, nb_iterations: int = 50) -> list:
    """
    Compute the errors between dynamic and static control points.

    Parameters
    ----------
    static_file_path : str
        The path to the file containing the static control points. e.g. theodolite_reference_prisms.txt
    dynamic_file_path : str
        The path to the file containing the dynamic control points.
    training_threshold : float
        The threshold used to split the dynamic control points into the training and prediction dataset. Default = 0.75
    nb_iterations : int
        The number of iterations. Default = 50

    Returns
    -------
    errors : list
        Returns a list containing all the errors, both for the dynamic and static control points.
        The first list holds the errors for the dynamic control points and the second one for the static control points.
    """
    ts1_static, ts2_static, ts3_static, T1_static, T12_static, T13_static = tu.read_marker_file(
        file_name=static_file_path, theodolite_reference_frame=1)
    ts1_dynamic = tu.read_prediction_data_resection_csv_file(dynamic_file_path + "_1.csv")[:, 1:].T
    ts2_dynamic = tu.read_prediction_data_resection_csv_file(dynamic_file_path + "_2.csv")[:, 1:].T
    ts3_dynamic = tu.read_prediction_data_resection_csv_file(dynamic_file_path + "_3.csv")[:, 1:].T

    nb_points = ts1_dynamic.shape[1]
    dynamic_errors = []
    static_errors = []

    for it in range(nb_iterations):
        mask = tu.uniform_random_mask(nb_points, threshold=training_threshold)

        training_1 = ts1_dynamic[:, mask]
        training_2 = ts2_dynamic[:, mask]
        training_3 = ts3_dynamic[:, mask]

        prediction_1 = ts1_dynamic[:, ~mask]
        prediction_2 = ts2_dynamic[:, ~mask]
        prediction_3 = ts3_dynamic[:, ~mask]

        T12_dynamic = tu.point_to_point_minimization(training_2, training_1)
        T13_dynamic = tu.point_to_point_minimization(training_3, training_1)

        prediction_2_dynamic = T12_dynamic @ prediction_2
        prediction_3_dynamic = T13_dynamic @ prediction_3

        for i, j, k in zip(prediction_1.T, prediction_2_dynamic.T, prediction_3_dynamic.T):
            dist_12 = np.linalg.norm(i[0:3] - j[0:3]) * 1000
            dist_13 = np.linalg.norm(i[0:3] - k[0:3]) * 1000
            dist_23 = np.linalg.norm(j[0:3] - k[0:3]) * 1000
            dynamic_errors.append(dist_12)
            dynamic_errors.append(dist_13)
            dynamic_errors.append(dist_23)

        prediction_2_static = T12_static @ prediction_2
        prediction_3_static = T13_static @ prediction_3

        for i, j, k in zip(prediction_1.T, prediction_2_static.T, prediction_3_static.T):
            dist_12 = np.linalg.norm(i[0:3] - j[0:3]) * 1000
            dist_13 = np.linalg.norm(i[0:3] - k[0:3]) * 1000
            dist_23 = np.linalg.norm(j[0:3] - k[0:3]) * 1000
            static_errors.append(dist_12)
            static_errors.append(dist_13)
            static_errors.append(dist_23)
    print(ts1_static, '\n', ts2_static, '\n', ts3_static, '\n', T1_static, '\n', T12_static, '\n', T13_static, '\n', T12_dynamic, '\n', T13_dynamic)
    return [dynamic_errors, static_errors]


def inter_prism_resection(Inter_distance, file_name_path, path_type, path_file_type, file_name_marker, rate,
                          prior, velocity_outlier, threshold_training, number_iteration):
    dist_prism_new_all = []
    dist_prism_basic_all = []
    error_prism_new_all = []
    error_prism_basic_all = []

    for i_dist, k_file, fm in zip(Inter_distance, file_name_path, file_name_marker):
        print(k_file)
        trimble_1 = tu.read_prediction_data_resection_csv_file(k_file + path_type + path_file_type + "1.csv")
        trimble_2 = tu.read_prediction_data_resection_csv_file(k_file + path_type + path_file_type + "2.csv")
        trimble_3 = tu.read_prediction_data_resection_csv_file(k_file + path_type + path_file_type + "3.csv")

        if (len(np.array(trimble_1)) > 0 and len(np.array(trimble_2)) > 0 and len(np.array(trimble_3)) > 0):
            speed_limit = velocity_outlier
            index1 = tu.find_not_moving_points_GP(np.array(trimble_1), speed_limit, 1/rate)

            p1 = np.array(trimble_1)[index1, 1:5]
            p2 = np.array(trimble_2)[index1, 1:5]
            p3 = np.array(trimble_3)[index1, 1:5]

            print(len(index))

            dist_prism_new = []
            dist_prism_basic = []
            error_new = []
            error_basic = []

            marker_1, marker_2, marker_3, T1_basic, T12_basic, T13_basic = tu.read_marker_file(fm, 1, 1)
            if(prior=="PTP"):
                T12_init = tu.point_to_point_minimization(p2.T, p1.T)
                T13_init = tu.point_to_point_minimization(p3.T, p1.T)
            else:
                if(prior=="CP"):
                    T12_init = T12_basic
                    T13_init = T13_basic

            T12_init_log = exp_inv_T(T12_init)
            T13_init_log = exp_inv_T(T13_init)

            x_init = [T12_init_log[2, 1], T12_init_log[0, 2], T12_init_log[1, 0], T12_init_log[0, 3],
                      T12_init_log[1, 3], T12_init_log[2, 3],
                      T13_init_log[2, 1], T13_init_log[0, 2], T13_init_log[1, 0], T13_init_log[0, 3],
                      T13_init_log[1, 3], T13_init_log[2, 3]]

            dist_12_t = i_dist[0]
            dist_13_t = i_dist[1]
            dist_23_t = i_dist[2]

            for num_it in range(0, number_iteration):
                print(num_it)
                mask = tu.random_splitting_mask(p1, threshold_training)
                p1_t = p1[mask]
                p2_t = p2[mask]
                p3_t = p3[mask]
                p1_p = p1[~mask]
                p2_p = p2[~mask]
                p3_p = p3[~mask]
                start_time = time.time()
                res = scipy.optimize.least_squares(lambda x: cost_fun_ls(p1_t,
                                                                         p2_t,
                                                                         p3_t,
                                                                         x[:6],
                                                                         x[6:],
                                                                         dist_12_t,
                                                                         dist_13_t,
                                                                         dist_23_t), x0=x_init, method='lm',
                                                   ftol=1e-15, xtol=1e-15, x_scale=1.0, loss='linear',
                                                   f_scale=1.0, diff_step=None, tr_solver=None, tr_options={},
                                                   jac_sparsity=None, max_nfev=30000000, verbose=2, args=(), kwargs={})
                stop_time = time.time()
                print(stop_time - start_time)
                xi_12 = res.x[:6]
                xi_13 = res.x[6:]
                T12 = exp_T(xi_12)
                T13 = exp_T(xi_13)
                T_1 = np.identity(4)
                p1t = (T_1@p1_p.T).T
                p2t = (T12@p2_p.T).T
                p3t = (T13@p3_p.T).T

                for i_n in range(0, len(p1t) - 1):
                    dp1 = abs(np.linalg.norm(p1t[i_n, 0:3] - p2t[i_n, 0:3]) - dist_12_t)*1000
                    dp2 = abs(np.linalg.norm(p1t[i_n, 0:3] - p3t[i_n, 0:3]) - dist_13_t)*1000
                    dp3 = abs(np.linalg.norm(p3t[i_n, 0:3] - p2t[i_n, 0:3]) - dist_23_t)*1000
                    dist_prism_new.append(dp1)
                    dist_prism_new.append(dp2)
                    dist_prism_new.append(dp3)

                    dp1 = abs(np.linalg.norm(p1_p[i_n, 0:3] - (T12_basic@p2_p[i_n, 0:4].T)[0:3]) - dist_12_t)*1000
                    dp2 = abs(np.linalg.norm(p1_p[i_n, 0:3] - (T13_basic@p3_p[i_n, 0:4].T)[0:3]) - dist_13_t)*1000
                    dp3 = abs(np.linalg.norm((T13_basic@p3_p[i_n, 0:4].T)[0:3] - (T12_basic@p2_p[i_n, 0:4].T)[
                                                                                 0:3]) - dist_23_t)*1000
                    dist_prism_basic.append(dp1)
                    dist_prism_basic.append(dp2)
                    dist_prism_basic.append(dp3)

                m1_b = marker_1
                m2_b = T12_basic@marker_2
                m3_b = T13_basic@marker_3

                m1_n = marker_1
                m2_n = T12@marker_2
                m3_n = T13@marker_3

                compute_error_between_points(m1_b, m2_b, m3_b, error_basic)
                compute_error_between_points(m1_n, m2_n, m3_n, error_new)

            dist_prism_new_all.append(dist_prism_new)
            dist_prism_basic_all.append(dist_prism_basic)
            error_prism_new_all.append(error_new)
            error_prism_basic_all.append(error_basic)

        else:
            print("No data in file(s) " + k + "  !!")
    print("Results done !")

    return dist_prism_new_all, dist_prism_basic_all, error_prism_new_all, error_prism_basic_all

def compute_error_between_points(m1_n, m2_n, m3_n, error):
    for i, j, k in zip(m1_n.T, m2_n.T, m3_n.T):
        dist_12 = np.linalg.norm(i[0:3] - j[0:3])*1000
        dist_13 = np.linalg.norm(i[0:3] - k[0:3])*1000
        dist_23 = np.linalg.norm(k[0:3] - j[0:3])*1000
        error.append(dist_12)
        error.append(dist_13)
        error.append(dist_23)

def geomatic_resection_optimization_on_pose(file_name, pilier_ref):
    _, _, _, trimble_1, trimble_2, trimble_3, _, _, _ = tu.read_marker_file_raw_data(file_name)
    subsets = [[1, 1, 0, 0],
           [1, 0, 1, 0],
           [1, 0, 0, 1],
           [0, 1, 1, 0],
           [0, 1, 0, 1],
           [0, 0, 1, 1]]
    method = 'TNC'
    tol = 1e-10
    maxfun = 500

    # x0 = [0, 0, 0, np.pi/2.]
    TF1 = tu.point_to_point_minimization(trimble_1, pilier_ref)
    R1 = TF1[0:3,0:3]
    eulerAngles1= R.from_matrix(R1).as_euler('xyz')
    x1 = [0, 0, 0, abs(eulerAngles1[2])]
    TF2 = tu.point_to_point_minimization(trimble_2, pilier_ref)
    R2 = TF2[0:3,0:3]
    eulerAngles2= R.from_matrix(R2).as_euler('xyz')
    x2 = [0, 0, 0, abs(eulerAngles1[2])]
    TF3 = tu.point_to_point_minimization(trimble_3, pilier_ref)
    R3 = TF3[0:3,0:3]
    eulerAngles3= R.from_matrix(R3).as_euler('xyz')
    x3 = [0, 0, 0, abs(eulerAngles3[2])]
    # print("x1: ", x1)
    # print("x2: ", x2)
    # print("x3: ", x3)
    
    error_all = []
    errors1 = []
    errors2 = []
    errors3 = []
    for sub in subsets:
        mask = np.array(sub)
        bmask = mask.astype(bool)
        pilier_t = pilier_ref[:, bmask]
        pilier_c = pilier_ref[:, ~bmask]
        ps1 = trimble_1[:, bmask]
        ps2 = trimble_2[:, bmask]
        ps3 = trimble_3[:, bmask]
        ps1_c = trimble_1[:, ~bmask]
        ps2_c = trimble_2[:, ~bmask]
        ps3_c = trimble_3[:, ~bmask]

        res = scipy.optimize.minimize(lambda X: cost_fun_one_pose(X, pilier_t, ps1), x0=x1, method=method, tol=tol,
                            options={'maxfun': maxfun})
        TW1 = T_z_so3(*res.x)
        res = scipy.optimize.minimize(lambda X: cost_fun_one_pose(X, pilier_t, ps2), x0=x2, method=method, tol=tol,
                            options={'maxfun': maxfun})
        TW2 = T_z_so3(*res.x)
        res = scipy.optimize.minimize(lambda X: cost_fun_one_pose(X, pilier_t, ps3), x0=x3, method=method, tol=tol,
                            options={'maxfun': maxfun})
        TW3 = T_z_so3(*res.x)

        # ============================================================
        # eval solution
        trimble_1_w = TW1@ps1_c
        trimble_2_w = TW2@ps2_c
        trimble_3_w = TW3@ps3_c
        # trimble_1_w = TW1@ps1
        # trimble_2_w = TW2@ps2
        # trimble_3_w = TW3@ps3
        compute_error_between_points(trimble_1_w, trimble_2_w, trimble_3_w, error_all)

        verrors1 = pilier_c - trimble_1_w
        for i in range(verrors1.shape[1]):
            errors1 += [np.linalg.norm(verrors1[:, i])*1000]

        verrors2 = pilier_c - trimble_2_w
        for i in range(verrors2.shape[1]):
            errors2 += [np.linalg.norm(verrors2[:, i])*1000]

        verrors3 = pilier_c - trimble_3_w
        for i in range(verrors3.shape[1]):
            errors3 += [np.linalg.norm(verrors3[:, i])*1000]
        
#         verrors1 = pilier_t - trimble_1_w
#         for i in range(verrors1.shape[1]):
#             errors1 += [np.linalg.norm(verrors1[:, i])*1000]

#         verrors2 = pilier_t - trimble_2_w
#         for i in range(verrors2.shape[1]):
#             errors2 += [np.linalg.norm(verrors2[:, i])*1000]

#         verrors3 = pilier_t - trimble_3_w
#         for i in range(verrors3.shape[1]):
#             errors3 += [np.linalg.norm(verrors3[:, i])*1000]

    return TW1,TW2,TW3,TW1@trimble_1, TW2@trimble_2, TW3@trimble_3, error_all, errors1, errors2, errors3
