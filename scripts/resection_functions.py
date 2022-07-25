import numpy as np

from scripts import theodolite_utils as tu


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

    return [dynamic_errors, static_errors]
