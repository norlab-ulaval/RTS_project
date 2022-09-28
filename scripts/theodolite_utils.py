import numpy as np
import math
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr
from pathlib import Path
from rosbags.typesys import get_types_from_msg, register_types
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R_scipy
from os.path import exists

# import rosbag
# import csv
# import random
# from numpy import linalg
# import sys
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import Vector3Stamped
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Float64MultiArray
# from scipy.interpolate import interp1d
# from matplotlib.patches import Ellipse
# from scipy import interpolate
# #from theodolite_node_msgs.msg import *
# #from theodolite_node_msgs.msg import TheodoliteCoordsStamped
# from std_msgs.msg import Header
# # from bagpy import bagreader
# from pathlib import PurePath

class TheodoliteCoordsStamped:
	def __init__(self, header, theodolite_time, theodolite_id, status, azimuth, elevation, distance):
		self.header = header
		self.theodolite_time = theodolite_time
		self.theodolite_id = theodolite_id
		self.status = status
		self.azimuth = azimuth
		self.elevation = elevation
		self.distance = distance

class TheodoliteTimeCorrection:
	def __init__(self, header, theodolite_id, estimated_time_offset):
		self.header = header
		self.theodolite_id = theodolite_id
		self.estimated_time_offset = estimated_time_offset

# ###################################################################################################
# ###################################################################################################
#
# Read/write data from files

def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)

def read_custom_messages():
	add_types = {}
	for pathstr in [
		'/home/maxime/workspace/src/theodolite_node_msgs/msg/TheodoliteCoordsStamped.msg',
		'/home/maxime/workspace/src/theodolite_node_msgs/msg/TheodoliteTimeCorrection.msg'
	]:
		msgpath = Path(pathstr)
		msgdef = msgpath.read_text(encoding='utf-8')
		add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))
	register_types(add_types)

def read_marker_file(file_name: str, theodolite_reference_frame: int, threshold: float = 1.0) -> tuple:
	"""
	Function to read a text file which contains the marker data for the calibration. The result given
	will be the different markers positions in one theodolite frame.

	It can also return a subsample of the markers position by providing a probability (prob) value less than one.

	The format of the file must be:
	theodolite_number , marker_number , status , elevation , azimuth , distance , sec , nsec

	Parameters
	----------
	file_name: Name of the file to read (the file should have the same structure then the usual one use by the raspi)
	theodolite_reference_frame: {1,2,3} Number which indicates the frame where the markers positions will be.
	threshold: Probability threshold at which a marker position is kept. (0, 1] optional

	Returns
	-------
	points_theodolite_1: list of array markers points coordinates of the theodolite 1, in the frame chosen
	points_theodolite_2: list of array markers points coordinates of the theodolite 2, in the frame chosen
	points_theodolite_3: list of array markers points coordinates of the theodolite 3, in the frame chosen
	T_.1: 4x4 rigid transform obtain according to the point-to-point minimization between the chosen frame and the
	theodolite 1 frame (Identity matrix if frame 1 chosen)
	T_.2: 4x4 rigid transform obtain according to the point-to-point minimization between the chosen frame and the
	theodolite 2 frame (Identity matrix if frame 2 chosen)
	T_.3: 4x4 rigid transform obtain according to the point-to-point minimization between the chosen frame and the
	theodolite 3 frame (Identity matrix if frame 3 chosen)
	"""
	assert theodolite_reference_frame == 1 or theodolite_reference_frame == 2 or theodolite_reference_frame == 3, \
		"Invalid theodolite_reference_frame value, it must be either 1, 2 or 3"
	assert 0.0 < threshold , \
		"Invalid probability threshold value, it must be greater than 0"

	points_theodolite_1 = []
	points_theodolite_2 = []
	points_theodolite_3 = []
	T_I = np.identity(4)

	with open(file_name, "r") as file:
		file.readline()

		for line in file:
			item = line.strip().split(" , ")
			if int(item[0]) == 1 and int(item[2]) == 0:
				add_point(float(item[5]), float(item[4]), float(item[3]), points_theodolite_1, 2)
			if int(item[0]) == 2 and int(item[2]) == 0:
				add_point(float(item[5]), float(item[4]), float(item[3]), points_theodolite_2, 2)
			if int(item[0]) == 3 and int(item[2]) == 0:
				add_point(float(item[5]), float(item[4]), float(item[3]), points_theodolite_3, 2)

	if(threshold<=1):
		probs = np.random.default_rng().uniform(size=len(points_theodolite_1))
		mask = (probs <= threshold)
		points_theodolite_1 = np.array(points_theodolite_1)[mask].T
		points_theodolite_2 = np.array(points_theodolite_2)[mask].T
		points_theodolite_3 = np.array(points_theodolite_3)[mask].T

		if theodolite_reference_frame == 1:
			T_12 = point_to_point_minimization(points_theodolite_2, points_theodolite_1)
			T_13 = point_to_point_minimization(points_theodolite_3, points_theodolite_1)
			return points_theodolite_1, points_theodolite_2, points_theodolite_3, T_I, T_12, T_13

		if theodolite_reference_frame == 2:
			T_21 = point_to_point_minimization(points_theodolite_1, points_theodolite_2)
			T_23 = point_to_point_minimization(points_theodolite_3, points_theodolite_2)
			return points_theodolite_1, points_theodolite_2, points_theodolite_3, T_21, T_I, T_23

		if theodolite_reference_frame == 3:
			T_31 = point_to_point_minimization(points_theodolite_1, points_theodolite_3)
			T_32 = point_to_point_minimization(points_theodolite_2, points_theodolite_3)
			return points_theodolite_1, points_theodolite_2, points_theodolite_3, T_31, T_32, T_I
	else:
		size = len(points_theodolite_1)

		assert 2 < size, \
			"Invalid number of control points, it must be greater than 2"
		assert 2 < round(threshold) < size, \
			"Invalid number of control points selected, it must be less than the number of control points and greater than 2"

		index = np.arange(size)
		mask = np.random.choice(index, size=round(threshold), replace=False)
		points_theodolite_1 = np.array(points_theodolite_1)[mask].T
		points_theodolite_2 = np.array(points_theodolite_2)[mask].T
		points_theodolite_3 = np.array(points_theodolite_3)[mask].T

		if theodolite_reference_frame == 1:
			T_12 = point_to_point_minimization(points_theodolite_2, points_theodolite_1)
			T_13 = point_to_point_minimization(points_theodolite_3, points_theodolite_1)
			return points_theodolite_1, points_theodolite_2, points_theodolite_3, T_I, T_12, T_13

		if theodolite_reference_frame == 2:
			T_21 = point_to_point_minimization(points_theodolite_1, points_theodolite_2)
			T_23 = point_to_point_minimization(points_theodolite_3, points_theodolite_2)
			return points_theodolite_1, points_theodolite_2, points_theodolite_3, T_21, T_I, T_23

		if theodolite_reference_frame == 3:
			T_31 = point_to_point_minimization(points_theodolite_1, points_theodolite_3)
			T_32 = point_to_point_minimization(points_theodolite_2, points_theodolite_3)
			return points_theodolite_1, points_theodolite_2, points_theodolite_3, T_31, T_32, T_I

def read_marker_file_raw_data(file_name: str):
    """
    Function to read a text file which contains the marker data for the calibration. The result given
    will be the raw data from each total station.

    The format of the file must be:
    theodolite_number , marker_number , status , elevation , azimuth , distance , sec , nsec

    Parameters
    ----------
    file_name: Name of the file to read (the file should have the same structure then the usual one use by the raspi)

    Returns
    -------
    raw_data_theodolite_1: nx3 list containing the elevation, the azimuth and the distance measurements of n markers in theodolite 1 frame
    raw_data_theodolite_2: nx3 list containing the elevation, the azimuth and the distance measurements of n markers in theodolite 2 frame
    raw_data_theodolite_3: nx3 list containing the elevation, the azimuth and the distance measurements of n markers in theodolite 3 frame
    points_theodolite_1: list of array markers points coordinates of the theodolite 1, in the frame chosen
    points_theodolite_2: list of array markers points coordinates of the theodolite 2, in the frame chosen
    points_theodolite_3: list of array markers points coordinates of the theodolite 3, in the frame chosen
    T_.1: 4x4 rigid transform obtain according to the point-to-point minimization between the chosen frame and the
    theodolite 1 frame (Identity matrix if frame 1 chosen)
    T_.2: 4x4 rigid transform obtain according to the point-to-point minimization between the chosen frame and the
    theodolite 2 frame (Identity matrix if frame 2 chosen)
    T_.3: 4x4 rigid transform obtain according to the point-to-point minimization between the chosen frame and the
    theodolite 3 frame (Identity matrix if frame 3 chosen)
    """
    points_theodolite_1 = []
    points_theodolite_2 = []
    points_theodolite_3 = []
    raw_data_theodolite_1 = []
    raw_data_theodolite_2 = []
    raw_data_theodolite_3 = []
    T_I = np.identity(4)
    correction = 0.01

    with open(file_name, "r") as file:
        file.readline()

        for line in file:
            item = line.strip().split(" , ")
            if int(item[0]) == 1 and int(item[2]) == 0:
                add_point(float(item[5]), float(item[4]), float(item[3]), points_theodolite_1, 2)
                raw_data_theodolite_1.append([float(item[3]), float(item[4]), float(item[5]) + correction])
            if int(item[0]) == 2 and int(item[2]) == 0:
                add_point(float(item[5]), float(item[4]), float(item[3]), points_theodolite_2, 2)
                raw_data_theodolite_2.append([float(item[3]), float(item[4]), float(item[5]) + correction])
            if int(item[0]) == 3 and int(item[2]) == 0:
                add_point(float(item[5]), float(item[4]), float(item[3]), points_theodolite_3, 2)
                raw_data_theodolite_3.append([float(item[3]), float(item[4]), float(item[5]) + correction])

    points_theodolite_1 = np.array(points_theodolite_1).T
    points_theodolite_2 = np.array(points_theodolite_2).T
    points_theodolite_3 = np.array(points_theodolite_3).T

    T_12 = point_to_point_minimization(points_theodolite_2, points_theodolite_1)
    T_13 = point_to_point_minimization(points_theodolite_3, points_theodolite_1)

    return raw_data_theodolite_1, raw_data_theodolite_2, raw_data_theodolite_3, points_theodolite_1, points_theodolite_2, points_theodolite_3, T_I, T_12, T_13

def read_rosbag_time_correction_theodolite(file):
	read_custom_messages()
	with Reader(file) as bag:
		timestamp_1 = []
		timeCorrection_1 = []
		timestamp_2 = []
		timeCorrection_2 = []
		timestamp_3 = []
		timeCorrection_3 = []
		# Read topic of trimble
		for connection, timestamp, rawdata in bag.messages():
			if connection.topic == '/theodolite_master/theodolite_correction_timestamp':
				msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
				marker = TheodoliteTimeCorrection(msg.header, msg.theodolite_id, msg.estimated_time_offset)
				if(marker.theodolite_id==1):
					timestamp_1.append(second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec))
					timeCorrection_1.append(second_nsecond(marker.estimated_time_offset.sec,marker.estimated_time_offset.nanosec))
				if (marker.theodolite_id == 2):
					timestamp_2.append(second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec))
					timeCorrection_2.append(second_nsecond(marker.estimated_time_offset.sec,marker.estimated_time_offset.nanosec))
				if (marker.theodolite_id == 3):
					timestamp_3.append(second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec))
					timeCorrection_3.append(second_nsecond(marker.estimated_time_offset.sec,marker.estimated_time_offset.nanosec))

	return timestamp_1, timestamp_2, timestamp_3, timeCorrection_1, timeCorrection_2, timeCorrection_3

# Function which read a rosbag of theodolite data and return the trajectories found by each theodolite, and the timestamp of each point as a list
# Input:
# - file: name of the rosbag to open
# - Tf: list of rigid transform between each frame according to the chosen one, was found according to the markers positions.
# Output:
# - trajectory_trimble_1: list of 4x1 3D homogeneous coordinates for the theodolite 1
# - trajectory_trimble_2: list of 4x1 3D homogeneous coordinates for the theodolite 2
# - trajectory_trimble_3: list of 4x1 3D homogeneous coordinates for the theodolite 2
# - time_trimble_1: list of timestamp for each points for the theodolite 1, timestamp in double
# - time_trimble_2: list of timestamp for each points for the theodolite 2, timestamp in double
# - time_trimble_3: list of timestamp for each points for the theodolite 3, timestamp in double
def read_rosbag_theodolite_with_tf(file, Tf):
	read_custom_messages()
	with Reader(file) as bag:
		trajectory_trimble_1=[]
		trajectory_trimble_2=[]
		trajectory_trimble_3=[]
		time_trimble_1 = []
		time_trimble_2 = []
		time_trimble_3 = []
		check_double_1 = 0
		check_double_2 = 0
		check_double_3 = 0
		# Variable for counting number of data and number of mistakes
		it = np.array([0,0,0])
		bad_measures = 0
		for connection, timestamp, rawdata in bag.messages():
			if connection.topic == '/theodolite_master/theodolite_data':
				msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
				marker = TheodoliteCoordsStamped(msg.header, msg.theodolite_time, msg.theodolite_id, msg.status, msg.azimuth, msg.elevation, msg.distance)
				if(marker.status == 0 and marker.distance<=1000): # If theodolite can see the prism, or no mistake in the measurement
					# Find number of theodolite
					if(marker.theodolite_id==1):
						if(check_double_1!=second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec)):
							add_point_in_frame(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_1, Tf[0], 2)
							time_trimble_1.append(second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec))
							it[0]+=1
							check_double_1 = second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec)
					if(marker.theodolite_id==2):
						if (check_double_2 != second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec)):
							add_point_in_frame(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_2, Tf[1], 2)
							time_trimble_2.append(second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec))
							it[1]+=1
							check_double_2 = second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec)
					if(marker.theodolite_id==3):
						if (check_double_3 != second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec)):
							add_point_in_frame(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_3, Tf[2], 2)
							time_trimble_3.append(second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec))
							it[2]+=1
							check_double_3 = second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec)
				# Count mistakes
				else:
					bad_measures+=1
	# Print number of data for each theodolite and the total number of mistakes
	print("Number of data for theodolites:", it)
	print("Bad measures:", bad_measures)

	sort_index1 = np.argsort(time_trimble_1)
	sort_index2 = np.argsort(time_trimble_2)
	sort_index3 = np.argsort(time_trimble_3)

	traj1 = np.array(trajectory_trimble_1)[sort_index1]
	traj2 = np.array(trajectory_trimble_2)[sort_index2]
	traj3 = np.array(trajectory_trimble_3)[sort_index3]
	tt1 = np.array(time_trimble_1)[sort_index1]
	tt2 = np.array(time_trimble_2)[sort_index2]
	tt3 = np.array(time_trimble_3)[sort_index3]

	return traj1, traj2, traj3, tt1, tt2, tt3

# # def read_rosbag_theodolite_with_tf_more(file, Tf):
# # 	bag = rosbag.Bag(file)
# # 	trajectory_trimble_1=[]
# # 	trajectory_trimble_2=[]
# # 	trajectory_trimble_3=[]
# # 	time_trimble_1 = []
# # 	time_trimble_2 = []
# # 	time_trimble_3 = []
# # 	distance_1 = []
# # 	distance_2 = []
# # 	distance_3 = []
#
# # 	# Variable for counting number of data and number of mistakes
# # 	it = np.array([0,0,0])
# # 	bad_measures = 0
# # 	#Read topic of trimble
# # 	for _, msg, t in bag.read_messages(topics=['/theodolite_master/theodolite_data']):
# # 		marker = TheodoliteCoordsStamped(msg.header, msg.theodolite_time, msg.theodolite_id, msg.status, msg.azimuth, msg.elevation, msg.distance)
# # 		timestamp = second_nsecond(marker.header.stamp.secs, marker.header.stamp.nsecs)
# # 		if(marker.status == 0): # If theodolite can see the prism, or no mistake in the measurement
# # 			# Find number of theodolite
# # 			if(marker.theodolite_id==1):
# # 				add_point_in_frame(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_1, Tf[0], 2)
# # 				time_trimble_1.append(timestamp)
# # 				distance_1.append(marker.distance)
# # 				it[0]+=1
# # 			if(marker.theodolite_id==2):
# # 				add_point_in_frame(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_2, Tf[1], 2)
# # 				time_trimble_2.append(timestamp)
# # 				distance_2.append(marker.distance)
# # 				it[1]+=1
# # 			if(marker.theodolite_id==3):
# # 				add_point_in_frame(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_3, Tf[2], 2)
# # 				time_trimble_3.append(timestamp)
# # 				distance_3.append(marker.distance)
# # 				it[2]+=1
# # 		# Count mistakes
# # 		if(marker.status != 0):
# # 			bad_measures+=1
# # 	# Print number of data for each theodolite and the total number of mistakes
# # 	print("Number of data for theodolites:", it)
# # 	print("Bad measures:", bad_measures)
#
# # 	sort_index1 = np.argsort(time_trimble_1)
# # 	sort_index2 = np.argsort(time_trimble_2)
# # 	sort_index3 = np.argsort(time_trimble_3)
#
# # 	traj1 = np.array(trajectory_trimble_1)[sort_index1]
# # 	traj2 = np.array(trajectory_trimble_2)[sort_index2]
# # 	traj3 = np.array(trajectory_trimble_3)[sort_index3]
# # 	tt1 = np.array(time_trimble_1)[sort_index1]
# # 	tt2 = np.array(time_trimble_2)[sort_index2]
# # 	tt3 = np.array(time_trimble_3)[sort_index3]
# # 	d1 = np.array(distance_1)[sort_index1]
# # 	d2 = np.array(distance_2)[sort_index2]
# # 	d3 = np.array(distance_3)[sort_index3]
#
# # 	return traj1, traj2, traj3, tt1, tt2, tt3, d1, d2, d3

def read_rosbag_theodolite_without_tf_raw_data_pre_filtered(file):
	read_custom_messages()
	with Reader(file) as bag:
		time_trimble_1 = []
		time_trimble_2 = []
		time_trimble_3 = []
		distance_1 = []
		distance_2 = []
		distance_3 = []
		azimuth_1 = []
		azimuth_2 = []
		azimuth_3 = []
		elevation_1 = []
		elevation_2 = []
		elevation_3 = []
		trajectory_trimble_1 = []
		trajectory_trimble_2 = []
		trajectory_trimble_3 = []
		check_double_1 = 0
		check_double_2 = 0
		check_double_3 = 0
		# Variable for counting number of data and number of mistakes
		it = np.array([0,0,0])
		bad_measures = 0
		#Read topic of trimble
		for connection, timestamp, rawdata in bag.messages():
			if connection.topic == '/theodolite_master/theodolite_data':
				msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
				marker = TheodoliteCoordsStamped(msg.header, msg.theodolite_time, msg.theodolite_id, msg.status, msg.azimuth, msg.elevation, msg.distance)
				timestamp = second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec)
				if(marker.status == 0): # If theodolite can see the prism, or no mistake in the measurement
					# Find number of theodolite
					if(marker.theodolite_id==1):
						if (check_double_1 != timestamp):
							add_point(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_1, 2)
							time_trimble_1.append(timestamp)
							distance_1.append(marker.distance)
							azimuth_1.append(marker.azimuth)
							elevation_1.append(marker.elevation)
							it[0]+=1
							check_double_1 = timestamp
					if(marker.theodolite_id==2):
						if (check_double_2 != timestamp):
							add_point(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_2, 2)
							time_trimble_2.append(timestamp)
							distance_2.append(marker.distance)
							azimuth_2.append(marker.azimuth)
							elevation_2.append(marker.elevation)
							it[1]+=1
							check_double_2 = timestamp
					if(marker.theodolite_id==3):
						if (check_double_3 != timestamp):
							add_point(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_3, 2)
							time_trimble_3.append(timestamp)
							distance_3.append(marker.distance)
							azimuth_3.append(marker.azimuth)
							elevation_3.append(marker.elevation)
							it[2]+=1
							check_double_3 = timestamp
				# Count mistakes
				if(marker.status != 0):
					bad_measures+=1
	# Print number of data for each theodolite and the total number of mistakes
	print("Number of data for theodolites:", it)
	print("Bad measures:", bad_measures)

	sort_index1 = np.argsort(time_trimble_1)
	sort_index2 = np.argsort(time_trimble_2)
	sort_index3 = np.argsort(time_trimble_3)

	t1 = np.array(time_trimble_1)[sort_index1]
	t2 = np.array(time_trimble_2)[sort_index2]
	t3 = np.array(time_trimble_3)[sort_index3]
	tp1 = np.array(trajectory_trimble_1)[sort_index1]
	tp2 = np.array(trajectory_trimble_2)[sort_index2]
	tp3 = np.array(trajectory_trimble_3)[sort_index3]
	d1 = np.array(distance_1)[sort_index1]
	d2 = np.array(distance_2)[sort_index2]
	d3 = np.array(distance_3)[sort_index3]
	a1 = np.array(azimuth_1)[sort_index1]
	a2 = np.array(azimuth_2)[sort_index2]
	a3 = np.array(azimuth_3)[sort_index3]
	e1 = np.array(elevation_1)[sort_index1]
	e2 = np.array(elevation_2)[sort_index2]
	e3 = np.array(elevation_3)[sort_index3]

	return t1, t2, t3, tp1, tp2, tp3, d1, d2, d3, a1, a2, a3, e1, e2, e3

def read_rosbag_theodolite_without_tf_raw_data(file):
	read_custom_messages()
	with Reader(file) as bag:
		time_trimble_1 = []
		time_trimble_2 = []
		time_trimble_3 = []
		distance_1 = []
		distance_2 = []
		distance_3 = []
		azimuth_1 = []
		azimuth_2 = []
		azimuth_3 = []
		elevation_1 = []
		elevation_2 = []
		elevation_3 = []
		trajectory_trimble_1 = []
		trajectory_trimble_2 = []
		trajectory_trimble_3 = []
		check_double_1 = 0
		check_double_2 = 0
		check_double_3 = 0
		# Variable for counting number of data and number of mistakes
		it = np.array([0,0,0])
		bad_measures = 0
		#Read topic of trimble
		for connection, timestamp, rawdata in bag.messages():
			if connection.topic == '/theodolite_master/theodolite_data':
				# print(connection)
				# print(timestamp)
				# print(rawdata)
				msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
				marker = TheodoliteCoordsStamped(msg.header, msg.theodolite_time, msg.theodolite_id, msg.status, msg.azimuth, msg.elevation, msg.distance)
				timestamp = second_nsecond(marker.header.stamp.sec, marker.header.stamp.nanosec)
				if(marker.status == 0): # If theodolite can see the prism, or no mistake in the measurement
					# Find number of theodolite
					if(marker.theodolite_id==1):
						if (check_double_1 != timestamp):
							add_point(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_1, 2)
							time_trimble_1.append(timestamp)
							distance_1.append(marker.distance)
							azimuth_1.append(marker.azimuth)
							elevation_1.append(marker.elevation)
							it[0]+=1
							check_double_1 = timestamp
					if(marker.theodolite_id==2):
						if (check_double_2 != timestamp):
							add_point(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_2, 2)
							time_trimble_2.append(timestamp)
							distance_2.append(marker.distance)
							azimuth_2.append(marker.azimuth)
							elevation_2.append(marker.elevation)
							it[1]+=1
							check_double_2 = timestamp
					if(marker.theodolite_id==3):
						if (check_double_3 != timestamp):
							add_point(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_3, 2)
							time_trimble_3.append(timestamp)
							distance_3.append(marker.distance)
							azimuth_3.append(marker.azimuth)
							elevation_3.append(marker.elevation)
							it[2]+=1
							check_double_3 = timestamp
				# Count mistakes
				if(marker.status != 0):
					bad_measures+=1
	# Print number of data for each theodolite and the total number of mistakes
	print("Number of data for theodolites:", it)
	print("Bad measures:", bad_measures)

	time_trimble_1 = np.array(time_trimble_1)
	time_trimble_2 = np.array(time_trimble_2)
	time_trimble_3 = np.array(time_trimble_3)
	trajectory_trimble_1 = np.array(trajectory_trimble_1).T
	trajectory_trimble_2 = np.array(trajectory_trimble_2).T
	trajectory_trimble_3 = np.array(trajectory_trimble_3).T
	distance_1 = np.array(distance_1)
	distance_2 = np.array(distance_2)
	distance_3 = np.array(distance_3)
	azimuth_1 = np.array(azimuth_1)
	azimuth_2 = np.array(azimuth_2)
	azimuth_3 = np.array(azimuth_3)
	elevation_1 = np.array(elevation_1)
	elevation_2 = np.array(elevation_2)
	elevation_3 = np.array(elevation_3)

	return time_trimble_1, time_trimble_2, time_trimble_3, trajectory_trimble_1, trajectory_trimble_2, trajectory_trimble_3, distance_1, distance_2, distance_3, azimuth_1, azimuth_2, azimuth_3, elevation_1, elevation_2, elevation_3

# # def read_rosbag_theodolite_without_tf_raw_data_all(file):
# # 	bag = rosbag.Bag(file)
# # 	time_trimble_1 = []
# # 	time_trimble_2 = []
# # 	time_trimble_3 = []
# # 	distance_1 = []
# # 	distance_2 = []
# # 	distance_3 = []
# # 	azimuth_1 = []
# # 	azimuth_2 = []
# # 	azimuth_3 = []
# # 	elevation_1 = []
# # 	elevation_2 = []
# # 	elevation_3 = []
# # 	status_1 = []
# # 	status_2 = []
# # 	status_3 = []
# # 	check_double_1 = 0
# # 	check_double_2 = 0
# # 	check_double_3 = 0
# # 	# Variable for counting number of data and number of mistakes
# # 	it = np.array([0,0,0])
# # 	bad_measures = 0
# # 	#Read topic of trimble
# # 	for _, msg, t in bag.read_messages(topics=['/theodolite_master/theodolite_data']):
# # 		marker = TheodoliteCoordsStamped(msg.header, msg.theodolite_time, msg.theodolite_id, msg.status, msg.azimuth, msg.elevation, msg.distance)
# # 		timestamp = second_nsecond(marker.header.stamp.secs, marker.header.stamp.nsecs)
# # 		# Find number of theodolite
# # 		if(marker.theodolite_id==1):
# # 			if (check_double_1 != timestamp):
# # 				#add_point_in_frame(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_1, Tf[0], 2)
# # 				time_trimble_1.append(timestamp)
# # 				distance_1.append(marker.distance)
# # 				azimuth_1.append(marker.azimuth)
# # 				elevation_1.append(marker.elevation)
# # 				status_1.append(marker.status)
# # 				it[0]+=1
# # 				check_double_1 = timestamp
# # 		if(marker.theodolite_id==2):
# # 			if (check_double_2 != timestamp):
# # 				#add_point_in_frame(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_2, Tf[1], 2)
# # 				time_trimble_2.append(timestamp)
# # 				distance_2.append(marker.distance)
# # 				azimuth_2.append(marker.azimuth)
# # 				elevation_2.append(marker.elevation)
# # 				status_2.append(marker.status)
# # 				it[1]+=1
# # 				check_double_2 = timestamp
# # 		if(marker.theodolite_id==3):
# # 			if (check_double_3 != timestamp):
# # 				#add_point_in_frame(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_3, Tf[2], 2)
# # 				time_trimble_3.append(timestamp)
# # 				distance_3.append(marker.distance)
# # 				azimuth_3.append(marker.azimuth)
# # 				elevation_3.append(marker.elevation)
# # 				status_3.append(marker.status)
# # 				it[2]+=1
# # 				check_double_3 = timestamp
# # 		# Count mistakes
# # 		if(marker.status != 0):
# # 			bad_measures+=1
# # 	# Print number of data for each theodolite and the total number of mistakes
# # 	print("Number of data for theodolites:", it)
# # 	print("Bad measures:", bad_measures)
#
# # 	sort_index1 = np.argsort(time_trimble_1)
# # 	sort_index2 = np.argsort(time_trimble_2)
# # 	sort_index3 = np.argsort(time_trimble_3)
#
# # 	tt1 = np.array(time_trimble_1)[sort_index1]
# # 	tt2 = np.array(time_trimble_2)[sort_index2]
# # 	tt3 = np.array(time_trimble_3)[sort_index3]
# # 	d1 = np.array(distance_1)[sort_index1]
# # 	d2 = np.array(distance_2)[sort_index2]
# # 	d3 = np.array(distance_3)[sort_index3]
# # 	a1 = np.array(azimuth_1)[sort_index1]
# # 	a2 = np.array(azimuth_2)[sort_index2]
# # 	a3 = np.array(azimuth_3)[sort_index3]
# # 	e1 = np.array(elevation_1)[sort_index1]
# # 	e2 = np.array(elevation_2)[sort_index2]
# # 	e3 = np.array(elevation_3)[sort_index3]
# # 	s1 = np.array(status_1)[sort_index1]
# # 	s2 = np.array(status_2)[sort_index2]
# # 	s3 = np.array(status_3)[sort_index3]
#
# # 	return tt1, tt2, tt3, d1, d2, d3, a1, a2, a3, e1, e2, e3, s1, s2, s3
#
# # def read_rosbag_theodolite_without_tf_raw_data_all_2(file):
# # 	bag = rosbag.Bag(file)
# # 	time_trimble_1 = []
# # 	time_trimble_2 = []
# # 	time_trimble_3 = []
# # 	distance_1 = []
# # 	distance_2 = []
# # 	distance_3 = []
# # 	azimuth_1 = []
# # 	azimuth_2 = []
# # 	azimuth_3 = []
# # 	elevation_1 = []
# # 	elevation_2 = []
# # 	elevation_3 = []
# # 	status_1 = []
# # 	status_2 = []
# # 	status_3 = []
# # 	# Variable for counting number of data and number of mistakes
# # 	it = np.array([0,0,0])
# # 	bad_measures = 0
# # 	#Read topic of trimble
# # 	for _, msg, t in bag.read_messages(topics=['/theodolite_master/theodolite_data']):
# # 		marker = TheodoliteCoordsStamped(msg.header, msg.theodolite_time, msg.theodolite_id, msg.status, msg.azimuth, msg.elevation, msg.distance)
# # 		timestamp = second_nsecond(marker.header.stamp.secs, marker.header.stamp.nsecs)
# # 		# Find number of theodolite
# # 		if(marker.theodolite_id==1):
# # 				#add_point_in_frame(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_1, Tf[0], 2)
# # 				time_trimble_1.append(timestamp)
# # 				distance_1.append(marker.distance)
# # 				azimuth_1.append(marker.azimuth)
# # 				elevation_1.append(marker.elevation)
# # 				status_1.append(marker.status)
# # 				it[0]+=1
# # 		if(marker.theodolite_id==2):
# # 				#add_point_in_frame(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_2, Tf[1], 2)
# # 				time_trimble_2.append(timestamp)
# # 				distance_2.append(marker.distance)
# # 				azimuth_2.append(marker.azimuth)
# # 				elevation_2.append(marker.elevation)
# # 				status_2.append(marker.status)
# # 				it[1]+=1
# # 		if(marker.theodolite_id==3):
# # 				#add_point_in_frame(marker.distance, marker.azimuth, marker.elevation, trajectory_trimble_3, Tf[2], 2)
# # 				time_trimble_3.append(timestamp)
# # 				distance_3.append(marker.distance)
# # 				azimuth_3.append(marker.azimuth)
# # 				elevation_3.append(marker.elevation)
# # 				status_3.append(marker.status)
# # 				it[2]+=1
# # 		# Count mistakes
# # 		if(marker.status != 0):
# # 			bad_measures+=1
# # 	# Print number of data for each theodolite and the total number of mistakes
# # 	print("Number of data for theodolites:", it)
# # 	print("Bad measures:", bad_measures)
#
# # 	return time_trimble_1, time_trimble_2, time_trimble_2, distance_1, distance_2, distance_3, azimuth_1, azimuth_2, azimuth_3, elevation_1, elevation_2, elevation_3, status_1, status_2, status_3
#
# # def read_rosbag_theodolite_without_tf(file):
# # 	bag = rosbag.Bag(file)
# # 	time_trimble_1 = []
# # 	time_trimble_2 = []
# # 	time_trimble_3 = []
# # 	trimble_1 = []
# # 	trimble_2 = []
# # 	trimble_3 = []
# # 	check_double_1 = 0
# # 	check_double_2 = 0
# # 	check_double_3 = 0
# # 	# Variable for counting number of data and number of mistakes
# # 	it = np.array([0, 0, 0])
# # 	bad_measures = 0
# # 	# Read topic of trimble
# # 	for _, msg, t in bag.read_messages(topics=['/theodolite_master/theodolite_data']):
# # 		marker = TheodoliteCoordsStamped(msg.header,
# # 										 msg.theodolite_time,
# # 										 msg.theodolite_id,
# # 										 msg.status,
# # 										 msg.azimuth,
# # 										 msg.elevation,
# # 										 msg.distance)
# # 		timestamp = second_nsecond(marker.header.stamp.secs, marker.header.stamp.nsecs)
# # 		if (marker.status == 0):  # If theodolite can see the prism, or no mistake in the measurement
# # 			# Find number of theodolite
# # 			if (marker.theodolite_id == 1):
# # 				if (check_double_1 != timestamp):
# # 					add_point(marker.distance, marker.azimuth, marker.elevation, trimble_1, 2)
# # 					time_trimble_1.append(timestamp)
# # 					it[0] += 1
# # 					check_double_1 = timestamp
# # 			if (marker.theodolite_id == 2):
# # 				if (check_double_2 != timestamp):
# # 					add_point(marker.distance, marker.azimuth, marker.elevation, trimble_2, 2)
# # 					time_trimble_2.append(timestamp)
# # 					it[1] += 1
# # 					check_double_2 = timestamp
# # 			if (marker.theodolite_id == 3):
# # 				if (check_double_3 != timestamp):
# # 					add_point(marker.distance, marker.azimuth, marker.elevation, trimble_3, 2)
# # 					time_trimble_3.append(timestamp)
# # 					it[2] += 1
# # 					check_double_3 = timestamp
# # 		# Count mistakes
# # 		if (marker.status != 0):
# # 			bad_measures += 1
# # 	# Print number of data for each theodolite and the total number of mistakes
# # 	print("Number of data for theodolites:", it)
# # 	print("Bad measures:", bad_measures)
#
# # 	sort_index1 = np.argsort(time_trimble_1)
# # 	sort_index2 = np.argsort(time_trimble_2)
# # 	sort_index3 = np.argsort(time_trimble_3)
#
# # 	tt1 = np.array(time_trimble_1)[sort_index1]
# # 	tt2 = np.array(time_trimble_2)[sort_index2]
# # 	tt3 = np.array(time_trimble_3)[sort_index3]
# # 	t1 = np.array(trimble_1)[sort_index1]
# # 	t2 = np.array(trimble_2)[sort_index2]
# # 	t3 = np.array(trimble_3)[sort_index3]
#
# # 	return tt1, tt2, tt3, t1, t2, t3
#
# # def read_rosbag_theodolite_without_tf_2(file):
# # 	bag = rosbag.Bag(file)
# # 	time_trimble_1 = []
# # 	time_trimble_2 = []
# # 	time_trimble_3 = []
# # 	trimble_1 = []
# # 	trimble_2 = []
# # 	trimble_3 = []
# # 	check_double_1 = 0
# # 	check_double_2 = 0
# # 	check_double_3 = 0
# # 	# Variable for counting number of data and number of mistakes
# # 	it = np.array([0, 0, 0])
# # 	bad_measures = 0
# # 	# Read topic of trimble
# # 	for _, msg, t in bag.read_messages(topics=['/theodolite_master/theodolite_data']):
# # 		marker = TheodoliteCoordsStamped(msg.header,
# # 										 msg.theodolite_time,
# # 										 msg.theodolite_id,
# # 										 msg.status,
# # 										 msg.azimuth,
# # 										 msg.elevation,
# # 										 msg.distance)
# # 		timestamp = second_nsecond(marker.header.stamp.secs, marker.header.stamp.nsecs)
# # 		if (marker.status == 0):  # If theodolite can see the prism, or no mistake in the measurement
# # 			# Find number of theodolite
# # 			if (marker.theodolite_id == 1):
# # 				if (check_double_1 != timestamp):
# # 					add_point(marker.distance, marker.azimuth, marker.elevation, trimble_1, 2)
# # 					time_trimble_1.append(timestamp)
# # 					it[0] += 1
# # 					check_double_1 = timestamp
# # 			if (marker.theodolite_id == 2):
# # 				if (check_double_2 != timestamp):
# # 					add_point(marker.distance, marker.azimuth, marker.elevation, trimble_2, 2)
# # 					time_trimble_2.append(timestamp)
# # 					it[1] += 1
# # 					check_double_2 = timestamp
# # 			if (marker.theodolite_id == 3):
# # 				if (check_double_3 != timestamp):
# # 					add_point(marker.distance, marker.azimuth, marker.elevation, trimble_3, 2)
# # 					time_trimble_3.append(timestamp)
# # 					it[2] += 1
# # 					check_double_3 = timestamp
# # 		# Count mistakes
# # 		if (marker.status != 0):
# # 			bad_measures += 1
# # 	# Print number of data for each theodolite and the total number of mistakes
# # 	print("Number of data for theodolites:", it)
# # 	print("Bad measures:", bad_measures)
#
# # 	return time_trimble_1, time_trimble_2, time_trimble_3, trimble_1, trimble_2, trimble_3
#
#
# # Function which read a rosbag of icp data and return the a list of the pose
# # Input:
# # - file: name of the rosbag to open
# # Output:
# # - pose: list of 4x4 pose matrix
# # - time_icp: list of timestamp for each pose
# # def read_rosbag_icp(filename):
# # 	file = filename + ".bag"
# # 	bag = rosbag.Bag(file)
# # 	pose = []
# # 	time_icp = []
# # 	for _, msg, t in bag.read_messages(topics=['/icp_odom']):
# # 		odom = Odometry(msg.header, msg.child_frame_id, msg.pose, msg.twist)
# # 		time = second_nsecond(odom.header.stamp.secs, odom.header.stamp.nsecs)
# # 		x=odom.pose.pose.position.x
# # 		y=odom.pose.pose.position.y
# # 		z=odom.pose.pose.position.z
# # 		qx=odom.pose.pose.orientation.x
# # 		qy=odom.pose.pose.orientation.y
# # 		qz=odom.pose.pose.orientation.z
# # 		qw=odom.pose.pose.orientation.w
# # 		T = np.identity(4)
# # 		r = R_scipy.from_quat([qx, qy, qz, qw])
# # 		Rot_r = r.as_matrix()
# # 		T[0:3,0:3]=Rot_r
# # 		T[0,3] = x
# # 		T[1,3] = y
# # 		T[2,3] = z
# # 		pose.append(T)
# # 		time_icp.append(time)
#
# # 	return pose, time_icp

# Function which read a csv file of points data with their timestamps
# Input:
# - file: name of the csv file to open
# Output:
# - Time: list of timestamp
# - data: list of array of axis value for points
def read_point_data_csv_file(file_name):
	Px = []
	Py = []
	Pz = []
	P1 = []
	Time = []
	data = []
	# Read text file
	file = open(file_name, "r")
	line = file.readline()
	while line:
		item = line.split(" ")
		Time.append(float(item[0]))
		Px.append(float(item[1]))
		Py.append(float(item[2]))
		Pz.append(float(item[3]))
		P1.append(1)
		array_point = np.array([float(item[1]), float(item[2]), float(item[3]), P1])
		data.append(array_point)
		line = file.readline()
	file.close()
	data_arr = np.array(data).T
	return Time, data_arr

# def read_point_data_csv_file_2(file_name):
# 	Px = []
# 	Py = []
# 	Pz = []
# 	P1 = []
# 	Time = []
# 	data = []
# 	# Read text file
# 	file = open(file_name, "r")
# 	line = file.readline()
# 	while line:
# 		item = line.split(" ")
# 		Time.append(float(item[0]))
# 		Px.append(float(item[1]))
# 		Py.append(float(item[2]))
# 		Pz.append(float(item[3]))
# 		P1.append(1)
# 		array_point = [float(item[1]), float(item[2]), float(item[3]), 1]
# 		data.append(array_point)
# 		line = file.readline()
# 	file.close()
# 	data_arr = np.array(data).T
# 	#data.append(Py)
# 	#data.append(Pz)
# 	#data.append(P1)
# 	return Time, data_arr

def read_prediction_data_GP_csv_file(file_name):
	data = []
	# Read text file
	file = open(file_name, "r")
	line = file.readline()
	while line:
		#line = line.replace("]","")
		item = line.replace("]","").replace("[","").split(" ")
		Time = float(item[0])
		Px = float(item[1])
		Py = float(item[2])
		Pz = float(item[3])
		C1 = float(item[4])
		C2 = float(item[5])
		C3 = float(item[6])
		array_point = np.array([Time, Px, Py, Pz, C1, C2, C3])
		data.append(array_point)
		line = file.readline()
	file.close()
	return data

def read_prediction_data_Linear_csv_file(file_name):
	data = []
	# Read text file
	file = open(file_name, "r")
	line = file.readline()
	while line:
		#line = line.replace("]","")
		item = line.replace("]","").replace("[","").split(" ")
		Time = float(item[0])
		Px = float(item[1])
		Py = float(item[2])
		Pz = float(item[3])
		array_point = np.array([Time, Px, Py, Pz, 1])
		data.append(array_point)
		line = file.readline()
	file.close()
	return data

def read_prediction_data_resection_csv_file(file_name: str, threshold: float = 1.0):
	data = []

	with open(file_name, "r") as file:
		for line in file:
			item = line.strip().split(" ")
			Time = float(item[0])
			Px = float(item[1])
			Py = float(item[2])
			Pz = float(item[3])
			array_point = np.array([Time, Px, Py, Pz, 1])
			data.append(array_point)

	prob = np.random.default_rng().uniform(size=len(data))
	mask = (prob <= threshold)

	return np.array(data)[mask]

def read_prediction_data_experiment_csv_file(file_name: str, threshold: float = 1.0):
	data = []

	with open(file_name, "r") as file:
		for line in file:
			item = line.strip().split(" ")
			Px = float(item[1])
			Py = float(item[2])
			Pz = float(item[3])
			array_point = np.array([Px, Py, Pz, 1])
			data.append(array_point)

	prob = np.random.default_rng().uniform(size=len(data))
	mask = (prob <= threshold)

	return np.array(data)[mask]

# def read_saved_tf(file_name):
# 	Tf = []
# 	with open(file_name, "r") as file:
# 		for line in file:
# 			item = line.strip().split(" ")
# 			T = np.identity(4)
# 			T[0, 0] = item[0]
# 			T[0, 1] = item[1]
# 			T[0, 2] = item[2]
# 			T[0, 3] = item[3]
# 			T[1, 0] = item[4]
# 			T[1, 1] = item[5]
# 			T[1, 2] = item[6]
# 			T[1, 3] = item[7]
# 			T[2, 0] = item[8]
# 			T[2, 1] = item[9]
# 			T[2, 2] = item[10]
# 			T[2, 3] = item[11]
# 			Tf.append(T)
#
# 	return Tf
#
# # Function which read a rosbag of odometry data and return the lists of the speed and acceleration data
# # Input:
# # - filename: name of the rosbag to open
# # - wheel: option to select the topic to read (True:/warthog_velocity_controller/odom, False:/imu_and_wheel_odom)
# # Output:
# # - speed: list of 1x2 matrix which contain the timestamp [0] and the speed [1] for each data
# # - accel: list of 1x2 matrix which contain the timestamp [0] and the accel [1] for each data
# # def read_rosbag_imu_node(filename, wheel):
# # 	bag = rosbag.Bag(filename)
# # 	speed = []
# # 	speed_only = []
# # 	time_only = []
# # 	accel = []
# # 	accel_only = []
# # 	if(wheel==True):
# # 		topic_name = '/warthog_velocity_controller/odom'
# # 	else:
# # 		topic_name = '/imu_and_wheel_odom'
# # 	for _, msg, t in bag.read_messages(topics=[topic_name]):
# # 		odom = Odometry(msg.header, msg.child_frame_id, msg.pose, msg.twist)
# # 		time = second_nsecond(odom.header.stamp.secs, odom.header.stamp.nsecs)
# # 		vitesse_lineaire = odom.twist.twist.linear.x
# # 		speed.append(np.array([time,vitesse_lineaire]))
# # 		speed_only.append(abs(vitesse_lineaire))
# # 		time_only.append(time)
# # 	speed_only_arr = np.array(speed_only)
# # 	time_only_arr = np.array(time_only)
# # 	diff_speed = np.diff(speed_only_arr)
# # 	time_diff_mean = np.mean(np.diff(time_only_arr), axis=0)
# # 	for i in range(0, len(diff_speed)):
# # 		accel.append(np.array([time_only[i],diff_speed[i]/time_diff_mean]))
# # 		accel_only.append(abs(diff_speed[i]/time_diff_mean))
# # 	return speed, accel, speed_only, accel_only
#
# # Function which read a rosbag of imu data and return the list of the angular velocity around Z axis
# # Input:
# # - filename: name of the rosbag to open
# # - wheel: option to select the topic to read (True:/imu_data, False:/MTI_imu/data)
# # Output:
# # - speed: list of 1x2 matrix which contain the timestamp [0] and the angular velocity around Z axis [1] for each data
# # def read_rosbag_imu_data(filename, wheel):
# # 	bag = rosbag.Bag(filename)
# # 	angular_speed = []
# # 	angular_speed_only = []
# # 	if(wheel==True):
# # 		topic_name = '/imu/data' #topic_name = '/imu_data'
# # 	else:
# # 		topic_name = '/MTI_imu/data'
# # 	for _, msg, t in bag.read_messages(topics=[topic_name]):
# # 		imu = Imu(msg.header, msg.orientation, msg.orientation_covariance, msg.angular_velocity, msg.angular_velocity_covariance, msg.linear_acceleration, msg.linear_acceleration_covariance)
# # 		time = second_nsecond(imu.header.stamp.secs, imu.header.stamp.nsecs)
# # 		angular_velocity_z = imu.angular_velocity.z
# # 		angular_speed.append(np.array([time, angular_velocity_z]))
# # 		angular_speed_only.append(abs(angular_velocity_z))
# # 	return angular_speed, angular_speed_only
#
# # Function which read a rosbag of both GPS data and return the lists of the position data
# # Input:
# # - filename: name of the rosbag to open
# # - number_gps: number of GPS to read (1 or less: front only, 2 or more: front and back)
# # Output:
# # - gps_front: list of 1x4 array, [0]: timestamp, [1]: x position, [2]: y position, [3]: z position
# # - gps_back: list of 1x4 array, [0]: timestamp, [1]: x position, [2]: y position, [3]: z position
# # def read_rosbag_gps_odom(filename, number_gps):
# # 	bag = rosbag.Bag(filename)
# # 	gps_front = []
# # 	gps_back = []
# # 	for _, msg, t in bag.read_messages(topics=['/odom_utm_front']):
# # 		odom = Odometry(msg.header, msg.child_frame_id, msg.pose, msg.twist)
# # 		time = second_nsecond(odom.header.stamp.secs, odom.header.stamp.nsecs)
# # 		gps_position_x = odom.pose.pose.position.x
# # 		gps_position_y = odom.pose.pose.position.y
# # 		gps_position_z = odom.pose.pose.position.z
# # 		gps_front.append(np.array([time, gps_position_x, gps_position_y, gps_position_z]))
#
# # 	if(number_gps<=1):
# # 		return gps_front
# # 	else:
# # 		for _, msg, t in bag.read_messages(topics=['/odom_utm_back']):
# # 			odom = Odometry(msg.header, msg.child_frame_id, msg.pose, msg.twist)
# # 			time = second_nsecond(odom.header.stamp.secs, odom.header.stamp.nsecs)
# # 			gps_position_x = odom.pose.pose.position.x
# # 			gps_position_y = odom.pose.pose.position.y
# # 			gps_position_z = odom.pose.pose.position.z
# # 			gps_back.append(np.array([time, gps_position_x, gps_position_y, gps_position_z]))
# # 		return gps_front, gps_back

# Function which read the raw GPS file data and return the data read
# Print also the number of satellite seeing (mean, std, min, max)
# Input:
# - name_file: name of the rosbag to open
# - limit_compteur: number of line to skip to read at the begining of the file
# Output:
# - GPS_front_raw_data: list of 1x4 array, [0]: timestamp, [1]: latitude (deg), [2]: longitude(deg), [3]: height (m)
def read_gps_file(name_file, limit_compteur):
	fichier = open(name_file, "r")
	compteur = 0
	GPS_front_raw_data = []
	satellite = []
	for line in fichier:
		if(compteur>limit_compteur):
			array_split = line.split(" ")
			while("" in array_split):
				array_split.remove("")
			time = array_split[1]
			lat = array_split[2]
			long_i = array_split[3]
			height = array_split[4]
			time_split = time.split(":")
			time_sec = float(time_split[0].strip())*3600 + float(time_split[1].strip())*60 + float(time_split[2].strip())
			GPS_front_raw_data.append(np.array([time_sec, float(lat.strip()), float(long_i.strip()), float(height.strip())]))
			temporary_list=[]
			for i in line.split(" "):
				if(i!=''):
					temporary_list.append(i)
			#print(temporary_list)
			#if(float(temporary_list[6])==2):
				#print(compteur)
			satellite.append(float(temporary_list[6].strip()))
		compteur = compteur + 1
	fichier.close()
	print("Average satellite number:", round(np.mean(satellite),1), ", Std: ", round(np.std(satellite),1), ", Min :",  np.min(satellite),", Max :", np.max(satellite))
	return GPS_front_raw_data

# def read_gps_point_processed(name_file):
# 	fichier = open(name_file, "r")
# 	GPS_data = []
# 	for line in fichier:
# 		array_split = line.split(" ")
# 		while("" in array_split):
# 			array_split.remove("")
# 		time = array_split[0]
# 		lat = array_split[1]
# 		long_i = array_split[2]
# 		height = array_split[3]
# 		GPS_data.append(np.array([float(time.strip()), float(lat.strip()), float(long_i.strip()), float(height.strip())]))
# 	fichier.close()
# 	return GPS_data
#
# def read_gps_distance_processed(name_file):
# 	fichier = open(name_file, "r")
# 	GPS_data = []
# 	for line in fichier:
# 		array_split = line.split(" ")
# 		while("" in array_split):
# 			array_split.remove("")
# 		time = array_split[0]
# 		dist = array_split[1]
# 		GPS_data.append(np.array([float(time.strip()), float(dist.strip())]))
# 	fichier.close()
# 	return GPS_data

# def read_error_list_file(error_file: str):
#     error_cp = []
#     error_exp = []
#
#     error_cp = list(np.genfromtxt(error_file+'cp.csv', delimiter=','))
#     error_exp = list(np.genfromtxt(error_file+'experiment.csv', delimiter=","))
#
#     return error_cp, error_exp
#
# def read_error_list_file_alone(error_file: str):
# 	error_list = []
# 	error_list = list(np.genfromtxt(error_file, delimiter=','))
# 	return error_list
#
# def read_results_drop_outliers(file_name):
# 	with open(file_name, "r") as file:
# 		for line in file:
# 			item = line.strip().split(" ")
# 	return item

def read_extrinsic_calibration_results_file(path_file):
	if(path_file==''):
		return []
	else:
		list_values = list(np.genfromtxt(path_file, delimiter=' '))
		return list_values


# # Function which convert interpolated data pose into a specific format to use evo library
# # Input:
# # - interpolated_time: list of timestamp of the pose
# # - Pose_lidar: list of 4x4 matrix of the poses
# # - output: name of the file to create
# def grountruth_convert_for_eval(interpolated_time, Pose_lidar, output):
# 	groundtruth_file = open(output,"w+")
# 	iterator_lidar = 0
# 	for j in interpolated_time:
# 		for i in j:
# 			T = Pose_lidar[iterator_lidar]
# 			Rot = R_scipy.from_matrix(T[0:3,0:3])
# 			quat = Rot.as_quat()
# 			result = np.array([i, T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]])
# 			groundtruth_file.write(str(result[0]))
# 			groundtruth_file.write(" ")
# 			groundtruth_file.write(str(result[1]))
# 			groundtruth_file.write(" ")
# 			groundtruth_file.write(str(result[2]))
# 			groundtruth_file.write(" ")
# 			groundtruth_file.write(str(result[3]))
# 			groundtruth_file.write(" ")
# 			groundtruth_file.write(str(result[4]))
# 			groundtruth_file.write(" ")
# 			groundtruth_file.write(str(result[5]))
# 			groundtruth_file.write(" ")
# 			groundtruth_file.write(str(result[6]))
# 			groundtruth_file.write(" ")
# 			groundtruth_file.write(str(result[7]))
# 			groundtruth_file.write("\n")
# 			iterator_lidar = iterator_lidar+1
# 	groundtruth_file.close()
# 	print("Conversion done !")

def grountruth_GP_convert_for_eval(interpolated_time, Pose_lidar, output):
	groundtruth_file = open(output,"w+")
	iterator_lidar = 0
	for j in interpolated_time:
		T = Pose_lidar[iterator_lidar]
		Rot = R_scipy.from_matrix(T[0:3,0:3])
		quat = Rot.as_quat()
		result = np.array([j, T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]])
		groundtruth_file.write(str(result[0]))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(result[1]))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(result[2]))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(result[3]))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(result[4]))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(result[5]))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(result[6]))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(result[7]))
		groundtruth_file.write("\n")
		iterator_lidar = iterator_lidar+1
	groundtruth_file.close()
	print("Conversion done !")

def grountruth_GP_gps_convert_for_eval(interpolated_time, Pose_gps, output):
	groundtruth_file = open(output,"w+")
	iterator_lidar = 0
	for j in interpolated_time:
		T = Pose_gps[iterator_lidar]
		result = np.array([j, T[0], T[1], T[2]])
		groundtruth_file.write(str(result[0]))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(result[1]))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(result[2]))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(result[3]))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(0))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(0))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(0))
		groundtruth_file.write(" ")
		groundtruth_file.write(str(1))
		groundtruth_file.write("\n")
		iterator_lidar = iterator_lidar+1
	groundtruth_file.close()
	print("Conversion done !")

# def grountruth_GP_gps_convert_for_eval2(Pose_gps, output):
# 	groundtruth_file = open(output,"w+")
# 	for j in Pose_gps:
# 		groundtruth_file.write(str(j[0]))
# 		groundtruth_file.write(" ")
# 		groundtruth_file.write(str(j[1]))
# 		groundtruth_file.write(" ")
# 		groundtruth_file.write(str(j[2]))
# 		groundtruth_file.write(" ")
# 		groundtruth_file.write(str(j[3]))
# 		groundtruth_file.write(" ")
# 		groundtruth_file.write(str(0))
# 		groundtruth_file.write(" ")
# 		groundtruth_file.write(str(0))
# 		groundtruth_file.write(" ")
# 		groundtruth_file.write(str(0))
# 		groundtruth_file.write(" ")
# 		groundtruth_file.write(str(1))
# 		groundtruth_file.write("\n")
# 	groundtruth_file.close()
# 	print("Conversion done !")
#
# # Function which convert icp data pose into a specific format to use evo library
# # Input:
# # - time_icp: list of timestamp of the pose
# # - Pose_lidar: list of 4x4 matrix of the poses
# # - output: name of the file to create
# def icp_convert_for_eval(time_icp, Pose_lidar, output):
# 	icp_file = open(output,"w+")
# 	iterator_lidar = 0
# 	for i in time_icp:
# 		T = Pose_lidar[iterator_lidar]
# 		Rot = R_scipy.from_matrix(T[0:3,0:3])
# 		quat = Rot.as_quat()
# 		result = np.array([i, T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]])
# 		icp_file.write(str(result[0]))
# 		icp_file.write(" ")
# 		icp_file.write(str(result[1]))
# 		icp_file.write(" ")
# 		icp_file.write(str(result[2]))
# 		icp_file.write(" ")
# 		icp_file.write(str(result[3]))
# 		icp_file.write(" ")
# 		icp_file.write(str(result[4]))
# 		icp_file.write(" ")
# 		icp_file.write(str(result[5]))
# 		icp_file.write(" ")
# 		icp_file.write(str(result[6]))
# 		icp_file.write(" ")
# 		icp_file.write(str(result[7]))
# 		icp_file.write("\n")
# 		iterator_lidar = iterator_lidar+1
# 	icp_file.close()
# 	print("Conversion done !")

def save_tf(tf1, tf2, tf3, output):
	file = open(output,"w+")
	tf = []
	tf.append(tf1)
	tf.append(tf2)
	tf.append(tf3)
	for i in tf:
		file.write(str(i[0, 0]))
		file.write(" ")
		file.write(str(i[0, 1]))
		file.write(" ")
		file.write(str(i[0, 2]))
		file.write(" ")
		file.write(str(i[0, 3]))
		file.write(" ")
		file.write(str(i[1, 0]))
		file.write(" ")
		file.write(str(i[1, 1]))
		file.write(" ")
		file.write(str(i[1, 2]))
		file.write(" ")
		file.write(str(i[1, 3]))
		file.write(" ")
		file.write(str(i[2, 0]))
		file.write(" ")
		file.write(str(i[2, 1]))
		file.write(" ")
		file.write(str(i[2, 2]))
		file.write(" ")
		file.write(str(i[2, 3]))
		file.write("\n")
	file.close()
	print("Conversion done !")

# Function which convert the raw data to a csv file
# Input:
# - time_data: array 1xN of time for the data (in seconds)
# - point_data: array of 3xN of the trajectory to save
# - file_name: string for the path and file name of the csv file
def Convert_raw_data_TS_with_GCP_calibration_to_csv(time_data, point_data, file_name):
	csv_file = open(file_name, "w+")
	for i,j in zip(time_data, point_data):
		csv_file.write(str(i))
		csv_file.write(" ")
		csv_file.write(str(j[0]))
		csv_file.write(" ")
		csv_file.write(str(j[1]))
		csv_file.write(" ")
		csv_file.write(str(j[2]))
		csv_file.write("\n")
	csv_file.close()
	print("Conversion done !")

def Convert_raw_data_point_to_csv(time_data, point_data, file_name):
	csv_file = open(file_name, "w+")
	for i,j in zip(time_data, point_data):
		csv_file.write(str(i))
		csv_file.write(" ")
		csv_file.write(str(j[0]))
		csv_file.write(" ")
		csv_file.write(str(j[1]))
		csv_file.write(" ")
		csv_file.write(str(j[2]))
		csv_file.write("\n")
	csv_file.close()
	print("Conversion done !")

def Convert_raw_data_GNSS_to_csv(time_data, point_data, file_name):
	csv_file = open(file_name, "w+")
	for i, j in zip(time_data, point_data):
		csv_file.write(str(i))
		csv_file.write(" ")
		csv_file.write(str(j[0]))
		csv_file.write(" ")
		csv_file.write(str(j[1]))
		csv_file.write(" ")
		csv_file.write(str(j[2]))
		csv_file.write("\n")
	csv_file.close()
	print("Conversion done !")

# def Convert_datane_to_csv(e_noise, filename_e):
# 	csv_file = open(filename_e, "w+")
# 	once = False
# 	for i in e_noise:
# 		if(once == False):
# 			csv_file.write(str(i[3][0]))
# 			csv_file.write("\n")
# 			once = True
# 		csv_file.write(str(i[0][0]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[0][1]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[0][2]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[1][0]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[1][1]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[1][2]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[2][0]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[2][1]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[2][2]))
# 		csv_file.write("\n")
# 	csv_file.close()
# 	print("Conversion done !")
#
# def Convert_datapr_to_csv(t, T, filename):
# 	csv_file = open(filename, "w+")
# 	for j,i in zip(t,T):
# 		csv_file.write(str(j))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[0,0]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[0,1]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[0,2]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[0,3]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[1,0]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[1,1]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[1,2]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[1,3]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[2,0]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[2,1]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[2,2]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[2,3]))
# 		csv_file.write("\n")
# 	csv_file.close()
# 	print("Conversion done !")
#
# def Convert_datant_to_csv(t_noise_1, t_noise_2, t_noise_3, filename_t):
# 	csv_file = open(filename_t, "w+")
# 	for i,j,k in zip(t_noise_1, t_noise_2, t_noise_3):
# 		csv_file.write(str(i))
# 		csv_file.write(" ")
# 		csv_file.write(str(j))
# 		csv_file.write(" ")
# 		csv_file.write(str(k))
# 		csv_file.write("\n")
# 	csv_file.close()
# 	print("Conversion done !")
#
# def Convert_data_prediction_to_csv(time_data, prediction, file_name):
# 	csv_file = open(file_name, "w+")
# 	for i,j in zip(time_data, prediction):
# 		csv_file.write(str(i))
# 		csv_file.write(" ")
# 		csv_file.write(str(j[1]))
# 		csv_file.write(" ")
# 		csv_file.write(str(j[2]))
# 		csv_file.write(" ")
# 		csv_file.write(str(j[3]))
# 		csv_file.write(" ")
# 		csv_file.write(str(j[4]))
# 		csv_file.write(" ")
# 		csv_file.write(str(j[5]))
# 		csv_file.write(" ")
# 		csv_file.write(str(j[6]))
# 		csv_file.write("\n")
# 	csv_file.close()
# 	print("Conversion done !")

# Function which convert the inter-GPS distance to a csv file
# Input:
# - time_data: array 1xN of time for the data (in seconds)
# - distance: array of 1xN of the inter-GPS distance (m)
# - file_name: string for the path and file name of the csv file
def Convert_inter_distance_GNSS_to_csv(time_data, distance, file_name):
	csv_file = open(file_name, "w+")
	for i,j in zip(time_data, distance):
		csv_file.write(str(i))
		csv_file.write(" ")
		csv_file.write(str(j))
		csv_file.write("\n")
	csv_file.close()
	print("Conversion done !")


# def Convert_gps_ape_error(gps_data, gps_ape, file_name):
# 	csv_file = open(file_name, "w+")
# 	for i,j,k in zip(gps_data.positions_xyz,gps_data.timestamps, gps_ape):
# 		csv_file.write(str(j))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[0]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[1]))
# 		csv_file.write(" ")
# 		csv_file.write(str(i[2]))
# 		csv_file.write(" ")
# 		csv_file.write(str(k))
# 		csv_file.write("\n")
# 	csv_file.close()
# 	print("Conversion done !")
#
# Function which convert the raw GPS data latitude and longitude to x,y in UTM frame
# Input:
# - Lat: latitude in deg
# - Long: longitude in deg
# Output:
# - UTMEasting: corrected y position in UTM frame (m)
# - UTMNorthing: corrected x position in UTM frame (m)
def LLtoUTM(Lat, Long):
	RADIANS_PER_DEGREE = math.pi/180.0
	DEGREES_PER_RADIAN = 180.0/math.pi
	# WGS84 Parameters
	WGS84_A = 6378137.0;
	WGS84_B = 6356752.31424518
	WGS84_F = 0.0033528107
	WGS84_E = 0.0818191908
	WGS84_EP = 0.0820944379
	# UTM Parameters
	UTM_K0 = 0.9996
	UTM_FE = 500000.0
	UTM_FN_N = 0.0
	UTM_FN_S = 10000000.0
	UTM_E2 = (WGS84_E*WGS84_E)
	UTM_E4 = (UTM_E2*UTM_E2)
	UTM_E6 = (UTM_E4*UTM_E2)
	UTM_EP2 = (UTM_E2/(1-UTM_E2))
	a = WGS84_A
	eccSquared = UTM_E2
	k0 = UTM_K0
	# Make sure the longitude is between -180.00 .. 179.9
	LongTemp = (Long+180)-int((Long+180)/360)*360-180
	LatRad = Lat*RADIANS_PER_DEGREE
	LongRad = LongTemp*RADIANS_PER_DEGREE
	ZoneNumber = int((LongTemp + 180)/6) + 1
	if( Lat >= 56.0 and Lat < 64.0 and LongTemp >= 3.0 and LongTemp < 12.0 ):
		ZoneNumber = 32
	# Special zones for Svalbard
	if( Lat >= 72.0 and Lat < 84.0 ):
		if(LongTemp >= 0.0  and LongTemp <  9.0 ):
			ZoneNumber = 31
		elif(LongTemp >= 9.0  and LongTemp < 21.0):
			ZoneNumber = 33
		elif(LongTemp >= 21.0 and LongTemp < 33.0):
			ZoneNumber = 35
		elif(LongTemp >= 33.0 and LongTemp < 42.0):
			ZoneNumber = 37
	# +3 puts origin in middle of zone
	LongOrigin = (ZoneNumber - 1)*6 - 180 + 3
	LongOriginRad = LongOrigin * RADIANS_PER_DEGREE
	# compute the UTM Zone from the latitude and longitude
	#snprintf(UTMZone, 4, "%d%c", ZoneNumber, UTMLetterDesignator(Lat))
	eccPrimeSquared = (eccSquared)/(1-eccSquared)
	N = a/math.sqrt(1-eccSquared * math.sin(LatRad) * math.sin(LatRad))
	T = math.tan(LatRad) * math.tan(LatRad)
	C = eccPrimeSquared * math.cos(LatRad) * math.cos(LatRad)
	A = math.cos(LatRad) * (LongRad-LongOriginRad)
	M = a*((1- eccSquared/4 - 3* eccSquared*eccSquared/64 - 5*eccSquared*eccSquared*eccSquared/256)*LatRad- (3*eccSquared/8	+ 3*eccSquared*eccSquared/32	+ 45*eccSquared*eccSquared*eccSquared/1024)*math.sin(2*LatRad)+ (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*math.sin(4*LatRad)- (35*eccSquared*eccSquared*eccSquared/3072)*math.sin(6*LatRad))
	UTMEasting = (k0*N*(A+(1-T+C)*A*A*A/6+ (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)+ 500000.0)
	UTMNorthing = (k0*(M+N*math.tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24+ (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)))
	if(Lat < 0):
		#10000000 meter offset for southern hemisphere
		UTMNorthing = UTMNorthing + 10000000.0
	return UTMEasting, UTMNorthing

# Function which convert the raw GPS data to UTM frame for each data read
# Input:
# - GPS_front_raw_data: list of 1x4 array raw data of the GPS, [0]: timestamp, [1]: latitude (deg), [2]: longitude(deg), [3]: height (m)
# - limit_data: array of 1x2 to specifiy which data to read and convert ([0]: index of begining [1]: index of end)
# - time_origin: boolean to set the time origin to zero (True) or to let it compute with the hour, minute and second ot the day (False)
# Output:
# - GPS_front_utm_data: list of 1x4 GPS data in UTM frame, [0]: timestamp, [1]: x(m), [2]: y(m), [3]: z(m)
def utm_gps_data(GPS_front_raw_data, limit_data, time_origin):
	GPS_front_utm_data = []
	compteur = 0
	origin_time = 0
	for i in GPS_front_raw_data:
		if(compteur == 0 and time_origin == True):
			origin_time = i[0]
		if(compteur >=limit_data[0] and compteur <= limit_data[1]):
			UTMEasting, UTMNorthing = LLtoUTM(i[1], i[2])
			GPS_front_utm_data.append(np.array([i[0] - origin_time + limit_data[2], UTMNorthing, UTMEasting, i[3]]))
		compteur = compteur + 1
	return GPS_front_utm_data

# Function which reads data coming from a calibration file and put them in another file
# Input:
# - file_name: string for the path and file name of the csv file
# def read_calibration_gps_prism(file_name, file_name_output):
# 	file = open(file_name, "r")
# 	line = file.readline()
# 	points = []
# 	number = 0
# 	while line:
# 		item = line.split(" ")
# 		ha = float(item[0])+float(item[1])*1/60+float(item[2])*1/3600
# 		va = float(item[6]) + float(item[7]) * 1 / 60 + float(item[8]) * 1 / 3600
# 		d = float(item[12])
# 		number = number + 1
# 		points.append(give_points_calibration(d, ha, va, 1))
# 		line = file.readline()
# 	file.close()
#
# 	dp12 = np.linalg.norm(points[0]-points[1], axis=0)
# 	dp13 = np.linalg.norm(points[0] - points[2], axis=0)
# 	dp23 = np.linalg.norm(points[1] - points[2], axis=0)
# 	dg12 = np.linalg.norm(points[3] - points[4], axis=0)
# 	dg13 = np.linalg.norm(points[3] - points[5], axis=0)
# 	dg23 = np.linalg.norm(points[4] - points[5], axis=0)
#
# 	print("Distance inter-prism [m]: ", dp12, dp13, dp23)
# 	print("Distance inter-GPS [m]: ", dg12, dg13, dg23)
#
# 	csv_file = open(file_name_output, "w+")
# 	csv_file.write(str(dp12))
# 	csv_file.write(" ")
# 	csv_file.write(str(dp13))
# 	csv_file.write(" ")
# 	csv_file.write(str(dp23))
# 	csv_file.write(" ")
# 	csv_file.write(str(dg12))
# 	csv_file.write(" ")
# 	csv_file.write(str(dg13))
# 	csv_file.write(" ")
# 	csv_file.write(str(dg23))
# 	csv_file.write("\n")
# 	csv_file.close()
#
# 	print("Conversion done !")

def read_calibration_gps_prism_lidar(file_name, file_name_output, name_lidar):
	file = open(file_name, "r")
	line = file.readline()
	points = []
	number = 0
	while line:
		item = line.split(" ")
		ha = float(item[0]) + float(item[1])*1/60 + float(item[2])*1/3600
		va = float(item[6]) + float(item[7])*1/60 + float(item[8])*1/3600
		d = float(item[12])
		number = number + 1
		points.append(give_points_calibration(d, ha, va, 1))
		line = file.readline()
	file.close()

	dp12 = np.linalg.norm(points[0] - points[1], axis=0)
	dp13 = np.linalg.norm(points[0] - points[2], axis=0)
	dp23 = np.linalg.norm(points[1] - points[2], axis=0)
	print("Distance inter-prism [m]: ", dp12, dp13, dp23)

	if(number>3):

		dg12 = np.linalg.norm(points[3] - points[4], axis=0)
		dg13 = np.linalg.norm(points[3] - points[5], axis=0)
		dg23 = np.linalg.norm(points[4] - points[5], axis=0)
		print("Distance inter-GPS [m]: ", dg12, dg13, dg23)

		if(number>6):

			if (name_lidar == "Robosense_32"):
				distance_lidar_top_to_lidar_origin = 0.063  # In meter, for RS32 on warthog

			l1 = points[6]
			l23 = points[7] - points[8]
			l4 = l1 - l23  # Vecteur directionnel lidar altitude
			l4n = 1 / np.linalg.norm(l4, axis=0)
			l = l1 - distance_lidar_top_to_lidar_origin * l4n

			csv_file = open(file_name_output, "w+")
			csv_file.write(str(dp12))
			csv_file.write(" ")
			csv_file.write(str(dp13))
			csv_file.write(" ")
			csv_file.write(str(dp23))
			csv_file.write(" ")
			csv_file.write(str(dg12))
			csv_file.write(" ")
			csv_file.write(str(dg13))
			csv_file.write(" ")
			csv_file.write(str(dg23))
			csv_file.write("\n")
			csv_file.close()

		else:

			csv_file = open(file_name_output, "w+")
			csv_file.write(str(dp12))
			csv_file.write(" ")
			csv_file.write(str(dp13))
			csv_file.write(" ")
			csv_file.write(str(dp23))
			csv_file.write(" ")
			csv_file.write(str(dg12))
			csv_file.write(" ")
			csv_file.write(str(dg13))
			csv_file.write(" ")
			csv_file.write(str(dg23))
			csv_file.write("\n")
			csv_file.close()

	else:

		csv_file = open(file_name_output, "w+")
		csv_file.write(str(dp12))
		csv_file.write(" ")
		csv_file.write(str(dp13))
		csv_file.write(" ")
		csv_file.write(str(dp23))
		csv_file.write("\n")
		csv_file.close()

	print("Conversion done !")

def read_calibration_prism_lidar_marmotte(file_name, file_name_output, name_lidar):
	file = open(file_name, "r")
	line = file.readline()
	points = []

	number = 0
	while line:
		item = line.split(" ")
		ha = float(item[0]) + float(item[1])*1/60 + float(item[2])*1/3600
		va = float(item[6]) + float(item[7])*1/60 + float(item[8])*1/3600
		d = float(item[12])  # check that for distance !
		number = number + 1
		points.append(give_points_calibration(d, ha, va, 1))
		line = file.readline()
	file.close()

	dp12 = np.linalg.norm(points[0] - points[1], axis=0)
	dp13 = np.linalg.norm(points[0] - points[2], axis=0)
	dp23 = np.linalg.norm(points[1] - points[2], axis=0)

	print("Distance inter-prism [m]: ", dp12, dp13, dp23)

	if(number>3):

		if (name_lidar == "Velodyne"):
			distance_lidar_origin_to_lidar_middle = 0.0378  # In meter, for Velodyne on Marmotte

		v1 = points[3][0:3]
		v2 = points[4][0:3]
		v3 = points[5][0:3]
		v4 = points[6][0:3]
		diff_23 = v2-v3
		diff_34 = v4-v3
		ns = np.cross(diff_23, diff_34)
		middle_plate = 0.5*(v2-v4)+v4

		origin_lidar = distance_lidar_origin_to_lidar_middle*1/(np.linalg.norm(v1-middle_plate))*(v1-middle_plate)+middle_plate
		lidar_x = 1/np.linalg.norm(-diff_23)*(-diff_23)
		lidar_z = 1/np.linalg.norm(ns)*(ns)
		lidar_y = np.cross(lidar_z,lidar_x)

		R = np.array([[lidar_x[0],lidar_y[0],lidar_z[0],origin_lidar[0]],
					  [lidar_x[1],lidar_y[1],lidar_z[1],origin_lidar[1]],
					  [lidar_x[2],lidar_y[2],lidar_z[2],origin_lidar[2]],
					  [0,0,0,1]])
		Rt = np.linalg.inv(R)
		p1 = Rt@points[0]
		p2 = Rt@points[1]
		p3 = Rt@points[2]

		csv_file = open(file_name_output, "w+")
		csv_file.write(str(dp12))
		csv_file.write(" ")
		csv_file.write(str(dp13))
		csv_file.write(" ")
		csv_file.write(str(dp23))
		csv_file.write("\n")
		csv_file.write(str(p1[0]))
		csv_file.write(" ")
		csv_file.write(str(p1[1]))
		csv_file.write(" ")
		csv_file.write(str(p1[2]))
		csv_file.write("\n")
		csv_file.write(str(p2[0]))
		csv_file.write(" ")
		csv_file.write(str(p2[1]))
		csv_file.write(" ")
		csv_file.write(str(p2[2]))
		csv_file.write("\n")
		csv_file.write(str(p3[0]))
		csv_file.write(" ")
		csv_file.write(str(p3[1]))
		csv_file.write(" ")
		csv_file.write(str(p3[2]))
		csv_file.write("\n")
		csv_file.close()

	else:
		csv_file = open(file_name_output, "w+")
		csv_file.write(str(dp12))
		csv_file.write(" ")
		csv_file.write(str(dp13))
		csv_file.write(" ")
		csv_file.write(str(dp23))
		csv_file.write("\n")
		csv_file.close()

	print("Conversion done !")

def save_error_list_to_file(errors: list, file_name: str):
    errors = np.array(errors)
    np.savetxt(file_name, errors, delimiter=" ")

def save_tf_list_to_file(TFs: list, file_name: str):
	output = file_name
	file = open(output,"w+")
	for i in TFs:
		file.write(str(i[0][0][0]))
		file.write(" ")
		file.write(str(i[0][0][1]))
		file.write(" ")
		file.write(str(i[0][0][2]))
		file.write(" ")
		file.write(str(i[0][0][3]))
		file.write(" ")
		file.write(str(i[0][1][0]))
		file.write(" ")
		file.write(str(i[0][1][1]))
		file.write(" ")
		file.write(str(i[0][1][2]))
		file.write(" ")
		file.write(str(i[0][1][3]))
		file.write(" ")
		file.write(str(i[0][2][0]))
		file.write(" ")
		file.write(str(i[0][2][1]))
		file.write(" ")
		file.write(str(i[0][2][2]))
		file.write(" ")
		file.write(str(i[0][2][3]))
		file.write("\n")
	file.close()

def save_tf_list_to_file_multi(TFs: list, file_name: str):
	output = file_name
	file = open(output,"w+")
	for i in TFs:
		file.write(str(i[0][0][0]))
		file.write(" ")
		file.write(str(i[0][0][1]))
		file.write(" ")
		file.write(str(i[0][0][2]))
		file.write(" ")
		file.write(str(i[0][0][3]))
		file.write(" ")
		file.write(str(i[0][1][0]))
		file.write(" ")
		file.write(str(i[0][1][1]))
		file.write(" ")
		file.write(str(i[0][1][2]))
		file.write(" ")
		file.write(str(i[0][1][3]))
		file.write(" ")
		file.write(str(i[0][2][0]))
		file.write(" ")
		file.write(str(i[0][2][1]))
		file.write(" ")
		file.write(str(i[0][2][2]))
		file.write(" ")
		file.write(str(i[0][2][3]))
		file.write("\n")
		file.write(str(i[1][0][0]))
		file.write(" ")
		file.write(str(i[1][0][1]))
		file.write(" ")
		file.write(str(i[1][0][2]))
		file.write(" ")
		file.write(str(i[1][0][3]))
		file.write(" ")
		file.write(str(i[1][1][0]))
		file.write(" ")
		file.write(str(i[1][1][1]))
		file.write(" ")
		file.write(str(i[1][1][2]))
		file.write(" ")
		file.write(str(i[1][1][3]))
		file.write(" ")
		file.write(str(i[1][2][0]))
		file.write(" ")
		file.write(str(i[1][2][1]))
		file.write(" ")
		file.write(str(i[1][2][2]))
		file.write(" ")
		file.write(str(i[1][2][3]))
		file.write("\n")
		file.write(str(i[2][0][0]))
		file.write(" ")
		file.write(str(i[2][0][1]))
		file.write(" ")
		file.write(str(i[2][0][2]))
		file.write(" ")
		file.write(str(i[2][0][3]))
		file.write(" ")
		file.write(str(i[2][1][0]))
		file.write(" ")
		file.write(str(i[2][1][1]))
		file.write(" ")
		file.write(str(i[2][1][2]))
		file.write(" ")
		file.write(str(i[2][1][3]))
		file.write(" ")
		file.write(str(i[2][2][0]))
		file.write(" ")
		file.write(str(i[2][2][1]))
		file.write(" ")
		file.write(str(i[2][2][2]))
		file.write(" ")
		file.write(str(i[2][2][3]))
		file.write("\n")
	file.close()

# def save_results_drop_outliers(file_name_path, param, results_arr):
# 	file_name = file_name_path + str(param[0]) + "-" + str(param[1]) + "-" + str(param[2]) + "-" + str(
# 			param[3]) + "-" + str(param[4]) + "-" + str(param[5]) + ".txt"
# 	file = open(file_name, "w+")
# 	file.write(str(results_arr[0]))
# 	file.write(" ")
# 	file.write(str(results_arr[1]))
# 	file.write(" ")
# 	file.write(str(results_arr[2]))
# 	file.write(" ")
# 	file.write(str(results_arr[3]))
# 	file.write(" ")
# 	file.write(str(results_arr[4]))
# 	file.write(" ")
# 	file.write(str(results_arr[5]))
# 	file.write("\n")
# 	file.close()
#
# ###################################################################################################
# ###################################################################################################
# # Process raw data from files
#
# Function to convert rosTime in seconds
# Input:
# - secs: Time seconds value
# - nsecs: Time nanoseconds value
# Output: seconds in double
def second_nsecond(secs, nsecs):
	return secs+nsecs*10**(-9)

# Function to return a point according to the data of the theodolite as array
# Input:
# - d: distance in meter
# - ha: horizontale angle
# - va: verticale angle
# - param: 1 use angle in degrees, param: 2 use angle in radians
# Ouput: 4x1 array with the 3D coordinates according to the data
def give_points(d, ha, va, param):
    d = d + 0.01 # add 10mm because measurements done by raspi
    if(param ==1):
        x=d*math.cos((90-ha)*np.pi/180)*math.sin(va*np.pi/180)
        y=d*math.sin((90-ha)*np.pi/180)*math.sin(va*np.pi/180)
        z=d*math.cos(va*np.pi/180)
    if(param ==2):
        x=d*math.cos(np.pi/2-ha)*math.sin(va)
        y=d*math.sin(np.pi/2-ha)*math.sin(va)
        z=d*math.cos(va)
    return np.array([x, y, z, 1],dtype=np.float64)

# def give_points_resection(d, ha, va, param):
#     d = d + 0.01 # add 10mm because measurements done by raspi
#     if(param ==1):
#         x=d*math.cos((360-ha)*np.pi/180)*math.cos((va-90)*np.pi/180)
#         y=d*math.sin((360-ha)*np.pi/180)*math.cos((va-90)*np.pi/180)
#         z=-d*math.sin((va-90)*np.pi/180)
#     if(param ==2):
#         x=d*math.cos((2*np.pi-ha))*math.cos(va-np.pi/2)
#         y=d*math.sin((2*np.pi-ha))*math.cos(va-np.pi/2)
#         z=-d*math.sin(va-np.pi/2)
#     return np.array([x, y, z, 1],dtype=np.float64)
#
# def give_points_without_correction(d, ha, va, param):
# 	d = d + 0
# 	if(param ==1):
# 		x=d*math.cos((-ha)*np.pi/180)*math.cos((90-va)*np.pi/180)
# 		y=d*math.sin((-ha)*np.pi/180)*math.cos((90-va)*np.pi/180)
# 		z=d*math.sin((90-va)*np.pi/180)
# 	if(param ==2):
# 		x=d*math.cos(-ha)*math.cos(np.pi/2-va)
# 		y=d*math.sin(-ha)*math.cos(np.pi/2-va)
# 		z=d*math.sin(np.pi/2-va)
# 	return np.array([x, y, z, 1],dtype=np.float64)

# 10 mm are taken into account during the measurements
def give_points_calibration(d, ha, va, param):
	if(param ==1):
		x=d*math.cos((-ha)*np.pi/180)*math.cos((90-va)*np.pi/180)
		y=d*math.sin((-ha)*np.pi/180)*math.cos((90-va)*np.pi/180)
		z=d*math.sin((90-va)*np.pi/180)
	if(param ==2):
		x=d*math.cos(-ha)*math.cos(np.pi/2-va)
		y=d*math.sin(-ha)*math.cos(np.pi/2-va)
		z=d*math.sin(np.pi/2-va)
	return np.array([x, y, z, 1],dtype=np.float64)

# def give_points_simulation(d, ha, va, param):
#     if(param ==1):
#         x=d*math.cos(ha*np.pi/180)*math.sin(va*np.pi/180)
#         y=d*math.sin(ha*np.pi/180)*math.sin(va*np.pi/180)
#         z=d*math.cos(va*np.pi/180)
#     if(param ==2):
#         x=d*math.cos(ha)*math.sin(va)
#         y=d*math.sin(ha)*math.sin(va)
#         z=d*math.cos(va)
#     return np.array([x, y, z, 1],dtype=np.float64)

# Function to convert a point according to the data of the theodolite into a frame according to a pose T,
# and put this point into a list of array
# Input:
# - d: distance in meter
# - ha: horizontale angle
# - va: verticale angle
# - points: list of points modified by the pose given
# - T: 4x4 pose matrix between the Frame of the point to the frame desired
# - param: 1 use angle in degrees, param: 2 use angle in radians
def add_point_in_frame(d, ha, va, points, T, param):
	vec = give_points(d, ha, va, param)
	vec_result = T@vec
	points.append(np.array([vec_result[0],vec_result[1],vec_result[2], 1],dtype=np.float64))
#
# Function to add a point in a list of array according to the data of the theodolite
# Input:
# - d: distance in meter
# - ha: horizontale angle
# - va: verticale angle
# - points: list of points which the result point will be add
# - param: 1 use angle in degrees, param: 2 use angle in radians
def add_point(d, ha, va, points, param):
	points.append(give_points(d, ha, va, param))
#
# def add_point_resection(d, ha, va, points, param):
# 	points.append(give_points_resection(d, ha, va, param))
#
# ###################################################################################################
# ###################################################################################################
# # Theodolite function for processing
#
# Function which found the tf transform between two point clouds using point-to-point minimization,
# where the matching was already done (mean each index of the point cloud for the reading and reference array match)
# Input:
# - P: the reading point cloud, can be 4xn or 3xn where n is the number of points
# - Q: the reference point cloud, can be 4xn or 3xn where n is the number of points
# Output: T a 4x4 pose matrix corresponding to the rigid transformation
def point_to_point_minimization(P, Q):
	# Errors at the beginning
	errors_before = Q - P
	# Centroide of each pointcloud
	mu_p = np.mean(P[0:3,:],axis=1)
	mu_q = np.mean(Q[0:3,:], axis=1)
	# Center each pointcloud
	P_mu = np.ones((3, P.shape[1]))
	Q_mu = np.ones((3, Q.shape[1]))
	for i in range(0,P_mu.shape[1]):
		P_mu[0:3,i] = P[0:3,i] - mu_p
	for i in range(0,Q_mu.shape[1]):
		Q_mu[0:3,i] = Q[0:3,i] - mu_q
	# Compute cross covariance matrix
	H = P_mu@Q_mu.T
	# Use SVD decomposition
	U, s, V = np.linalg.svd(H)
	# Compute rotation
	R = V.T@U.T
	if(np.linalg.det(R)<0):
		#print(V.T)
		V_t = V.T
		V_t[:,2] = -V_t[:,2]
		R = V_t@U.T

	# Compute translation
	t = mu_q - R@mu_p
	# Compute rigid transformation obtained
	T = np.eye(4)
	T[0:3,0:3]=R
	T[0:3,3] = t
	return T

# # Function to find prism not moving points according to the point just before in the array of the trajectories.
# # The not moving point are selected because of their position proximity
# # Input:
# # - trimble: list of trajectory points
# # - limit_m: proximity limit in meters to find not moving points. If the distance between two near indexed points is less than the limit,
# # the point at the index i is selected
# # Output: list of index of the not moving points
# def find_not_moving_points(trimble, limit_m):
# 	ind_not_moving = []
# 	start_point = trimble[0:3,0]
# 	for i in range(1,len(trimble.T)):
# 		if(np.linalg.norm(trimble[0:3,i]-start_point)<limit_m):
# 			ind_not_moving.append(i)
# 		start_point = trimble[0:3,i]
# 	return ind_not_moving
#
# # Function to find lidar interpolated not moving points
# # Input:
# # - pose_lidar: list of lidar pose 4x4 matrix
# # - limit_speed: threshold of the speed to consider a position as static (m/s)
# # - time_inter: time between each interpolated points (s)
# # Output:
# # - ind_not_moving: list of index of the not moving points
# def find_not_moving_points_lidar(pose_lidar, limit_speed, time_inter):
# 	ind_not_moving = []
# 	for i in range(1,len(pose_lidar)):
# 		if(np.linalg.norm(pose_lidar[i,0:3,3]-pose_lidar[i-1,0:3,3])/time_inter<limit_speed):
# 			ind_not_moving.append(i)
# 	return ind_not_moving

def find_not_moving_points_GP(pose, limit_speed, time_inter):
	ind_not_moving = []
	for i in range(1,len(pose)):
		if(np.linalg.norm(pose[i,1:4]-pose[i-1,1:4])/time_inter<limit_speed):
			ind_not_moving.append(i)
	return ind_not_moving

# # Function to find lidar interpolated moving points
# # Input:
# # - pose_lidar: list of lidar pose 4x4 matrix
# # - limit_speed: threshold of the speed to consider a position as dynamic (m/s)
# # - time_inter: time between each interpolated points (s)
# # Output:
# # - ind_not_moving: list of index of the moving points
# def find_moving_points_lidar(pose_lidar, limit_speed, time_inter):
# 	ind_not_moving = []
# 	for i in range(1,len(pose_lidar)):
# 		if(np.linalg.norm(pose_lidar[i,0:3,3]-pose_lidar[i-1,0:3,3])/time_inter>=limit_speed):
# 			speed = np.linalg.norm(pose_lidar[i,0:3,3]-pose_lidar[i-1,0:3,3])/time_inter
# 			ind_not_moving.append(np.array([i,speed]))
# 	return ind_not_moving
#
# # Function to find cluster of not moving point
# # Input:
# # - trimble_time: list of time trajectory points
# # - indice_trimble_list: list of index of not moving points in a trajectory
# # - limit_time: proximity limit in second to find not moving points. If the time between two near indexed points is less than the limit,
# # these points are selected
# # Output:
# # - tuple_not_moving: list of array, each of this array contain the index of each cluster of not moving points in a trajectory
# def find_cluster_not_moving_points(trimble_time, indice_trimble_list, limit_time):
# 	tuple_not_moving = []
# 	list_temporary = []
# 	start_cluster = 0
# 	for i in range(0,len(indice_trimble_list)-1):
# 		if(abs(trimble_time[indice_trimble_list[i+1]]-trimble_time[indice_trimble_list[i]])<=limit_time):
# 			list_temporary.append(indice_trimble_list[i])
# 		if((abs(trimble_time[indice_trimble_list[i+1]]-trimble_time[indice_trimble_list[i]])>limit_time or i==len(indice_trimble_list)-2) and len(list_temporary)>0):
# 			tuple_not_moving.append(np.array(list_temporary))
# 			if(len(list_temporary)<5):
# 				del tuple_not_moving[-1]
# 			list_temporary = []
# 	return tuple_not_moving
#
# # Function to find cluster of interpolated lidar not moving point
# # Input:
# # - lidar_time: list of time trajectory points
# # - indice_lidar_list: list of index of lidar not moving points in a trajectory
# # - limit_time: proximity limit in second to find not moving points. If the time between two near indexed points is less than the limit,
# # these points are selected
# # Output:
# # - tuple_not_moving: list of array, each of this array contain the index of each cluster of not moving points in a trajectory
# def find_cluster_not_moving_points_lidar(lidar_time, indice_lidar_list, limit_time):
# 	tuple_not_moving = []
# 	list_temporary = []
# 	start_cluster = 0
# 	for i in range(0,len(indice_lidar_list)-1):
# 		if(abs(lidar_time[indice_lidar_list[i+1]]-lidar_time[indice_lidar_list[i]])<=limit_time):
# 			list_temporary.append(indice_lidar_list[i])
# 		if(abs(lidar_time[indice_lidar_list[i+1]]-lidar_time[indice_lidar_list[i]])>limit_time or i==len(indice_lidar_list)-2):
# 			tuple_not_moving.append(np.array(list_temporary))
# 			list_temporary = []
# 	return tuple_not_moving

# Function to split a time array into several interval according to a limit between two timestamp
# Input:
# - time_trimble: list of time (s)
# - limit_time_interval: threshold which is used to split the time interval in two if the timestamp difference is too high
# Output:
# - list_time_interval: list of 1x2 array of the different index which defined each of the intervals, [0]: begin and [1]: end
def split_time_interval(time_trimble, limit_time_interval):
	list_time_interval = []
	begin = 0
	min_number_points = 6
	max_number_points = 500

	for i in range(1,len(time_trimble)):
		if abs(time_trimble[i] - time_trimble[i-1]) > limit_time_interval or i == len(time_trimble)-1:
			number_points = i-begin

			if (min_number_points < number_points <= max_number_points) or (number_points > max_number_points and number_points / max_number_points <= 1.5):
				last = i if i== len(time_trimble)-1 else i-1
				interval = np.array([begin, last])
				begin = i
				list_time_interval.append(interval)
			elif number_points > max_number_points and number_points / max_number_points > 1.5:
				number_subintervals = math.ceil(number_points / max_number_points)
				subinterval_number_points = number_points // number_subintervals
				last_subinterval_number_points = number_points - subinterval_number_points*(number_subintervals-1)

				for j in range(number_subintervals-1):
					subinterval = np.array([begin, begin+subinterval_number_points-1])
					list_time_interval.append(subinterval)
					begin += subinterval_number_points

				last = last_subinterval_number_points if i == len(time_trimble)-1 else last_subinterval_number_points-1
				last_subinterval = np.array([begin, begin+last])
				list_time_interval.append(last_subinterval)
				begin = i
			else:
				begin = i

	return list_time_interval

# Function to find the closest index according to a timestamp in an simple list
# Input:
# - time_trimble: list of time (s)
# - time_interval: timestamp to find in the list (s)
# - limit_search: threshold of the time difference to use for the research (s)
# Output:
# - index: return the closest index found in the list, or -1 if there is not close index
def research_index_for_time(time_trimble, time_interval, limit_search):
	diff = limit_search
	index = 0
	found_one = 0
	for i in range(0,len(time_trimble)):
		if(abs(time_interval-time_trimble[i])< limit_search and diff > abs(time_interval-time_trimble[i])):
			diff = abs(time_interval - time_trimble[i])
			index = i
			found_one = 1
	if(found_one == 0):
		index = -1
	return index

# # Function to find the closest index according to a timestamp in a list of 1x2 array
# # Input:
# # - speed: list of data, [0] timestamp
# # - time_interval: timestamp to find in the list (s)
# # - limit_search: threshold of the time difference to use for the research (s)
# # Output:
# # - index: return the closest index found in the list, or -1 if there is not close index
# def research_index_for_time_speed(speed, time_interval, limit_search):
# 	result = 0
# 	diff = limit_search
# 	index = 0
# 	found_one = 0
# 	for i in range(0,len(speed)):
# 		if(abs(time_interval-speed[i][0])< limit_search and diff > abs(time_interval-speed[i][0])):
# 			diff = abs(time_interval-speed[i][0])
# 			result = speed[i][0]
# 			index = i
# 			found_one = 1
# 	if(found_one == 0):
# 		index = -1
# 	return index
#
#
# # Returns element closest to target in an array
# # Input:
# # - arr: array of data 1xN, timestamp (s)
# # - target: timestamp to find in arr (s)
# # Output:
# # - index: return the closest index found in arr and the value
# def findClosest(arr, target):
# 	n = len(arr)
# 	# Corner cases
# 	if (target <= arr[0]):
# 		return 0, arr[0]
# 	if (target >= arr[n - 1]):
# 		return n - 1, arr[n - 1]
#
# 	# Doing binary search
# 	i = 0
# 	j = n
# 	mid = 0
# 	while (i < j):
# 		mid = (i + j) // 2
# 		if (arr[mid] == target):
# 			return mid, arr[mid]
# 		# If target is less than array
# 		# element, then search in left
# 		if (target < arr[mid]):
# 			# If target is greater than previous
# 			# to mid, return closest of two
# 			if (mid > 0 and target > arr[mid - 1]):
# 				return mid, getClosest(arr[mid - 1], arr[mid], target)
# 			# Repeat for left half
# 			j = mid
# 		# If target is greater than mid
# 		else:
# 			if (mid < n - 1 and target < arr[mid + 1]):
# 				return mid, getClosest(arr[mid], arr[mid + 1], target)
# 			# update i
# 			i = mid + 1
# 	# Only single element left after search
# 	return mid, arr[mid]
#
# # Method to compare which one is the more close.
# # We find the closest by taking the difference
# # between the target and both values. It assumes
# # that val2 is greater than val1 and target lies
# # between these two.
# def getClosest(val1, val2, target):
# 	if (target - val1 >= val2 - target):
# 		return val2
# 	else:
# 		return val1
#
# # Function to compute the cluster of not moving points from the prisms according to the spacial and time distance
# # Input:
# # - trimble_1, trimble_2, trimble_3: list of prism positions for each theodolite
# # - time_trimble_1, time_trimble_2, time_trimble_3: list of time for each prism position
# # - dist_max: distance max for a prism to be part of a cluster
# # - time_max: time max for a prism to be part of a cluster
# # Output:
# # - tuple_not_moving_trimble_1, tuple_not_moving_trimble_2, tuple_not_moving_trimble_3: list of index cluster for not moving points
# def cluster_not_moving_points(trimble_1, trimble_2, trimble_3, time_trimble_1, time_trimble_2, time_trimble_3, dist_max, time_max):
# 	# find cluster by distance
# 	not_moving_trimble_1 = find_not_moving_points(trimble_1, dist_max)
# 	not_moving_trimble_2 = find_not_moving_points(trimble_2, dist_max)
# 	not_moving_trimble_3 = find_not_moving_points(trimble_3, dist_max)
# 	# sort these cluster with the time
# 	tuple_not_moving_trimble_1 = find_cluster_not_moving_points(time_trimble_1, not_moving_trimble_1, time_max)
# 	tuple_not_moving_trimble_2 = find_cluster_not_moving_points(time_trimble_2, not_moving_trimble_2, time_max)
# 	tuple_not_moving_trimble_3 = find_cluster_not_moving_points(time_trimble_3, not_moving_trimble_3, time_max)
# 	return tuple_not_moving_trimble_1, tuple_not_moving_trimble_2, tuple_not_moving_trimble_3
#
# # Function to compute the distance between prisms for the not moving points cluster
# # Input:
# # - interpolated_trajectories: list of the different interpolated trajectories
# # - tuple_not_moving_lidar: list the index of the tuple not moving points
# # Output:
# # - distance_error_12, distance_error_13, distance_error_23: list of index cluster for not moving points
# def not_moving_interpolated_points(interpolated_trajectories, tuple_not_moving_lidar):
# 	interpolated_prism_pose = []
# 	# concatenate all the sub-trajectories in one for each prism
# 	interpolated_prism_pose = interpolated_trajectories[0]
# 	for i in range(1,len(interpolated_trajectories)):
# 		interpolated_prism_pose[0] = np.concatenate((interpolated_prism_pose[0],interpolated_trajectories[i][0]), axis=1)
# 		interpolated_prism_pose[1] = np.concatenate((interpolated_prism_pose[1],interpolated_trajectories[i][1]), axis=1)
# 		interpolated_prism_pose[2] = np.concatenate((interpolated_prism_pose[2],interpolated_trajectories[i][2]), axis=1)
# 	# With the cluster index, find the mean of the prism position for each of the cluster
# 	mean_not_moving_trimble_1 = []
# 	mean_not_moving_trimble_2 = []
# 	mean_not_moving_trimble_3 = []
# 	for i in tuple_not_moving_lidar:
# 		mean_not_moving_trimble_1.append(np.mean(interpolated_prism_pose[0][:,i],axis=1))
# 		mean_not_moving_trimble_2.append(np.mean(interpolated_prism_pose[1][:,i],axis=1))
# 		mean_not_moving_trimble_3.append(np.mean(interpolated_prism_pose[2][:,i],axis=1))
# 	# calculate the inter-prism distance for each of the cluster
# 	distance_error_12 = []
# 	distance_error_13 = []
# 	distance_error_23 = []
# 	for i,j,k in zip(mean_not_moving_trimble_1,mean_not_moving_trimble_2,mean_not_moving_trimble_3):
# 		distance_error_12.append(abs(np.linalg.norm(i-j)-np.linalg.norm(Prism_1-Prism_2))*1000)
# 		distance_error_13.append(abs(np.linalg.norm(k-j)-np.linalg.norm(Prism_3-Prism_2))*1000)
# 		distance_error_23.append(abs(np.linalg.norm(i-k)-np.linalg.norm(Prism_1-Prism_3))*1000)
# 	return distance_error_12, distance_error_13, distance_error_23
#
# ###################################################################################################
# ###################################################################################################
# # Function for plot
#
# # Function to find the eigenvectors and eigenvalues of a covariance matrix
# # Input: covariance matrix
# # Output:
# # - vals: eigenvalues
# # - vecs: eigenvectors
# def eigsorted(cov):
# 	vals, vecs = np.linalg.eigh(cov)
# 	order = vals.argsort()[::-1]
# 	return vals[order], vecs[:,order]
#
# # Function to find the parameter to plot on ellipse in 2D
# # Input:
# # - cov: covariance matrix of the point cloud
# # Output:
# # - width: width of the ellipse
# # - height: height of the ellipse
# # - theta: angle between the two eigenvectors
# def cov_ellipse(cov):
# 	vals, vecs = eigsorted(cov)
# 	theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
# 	# Width and height are "full" widths, not radius
# 	width, height = np.sqrt(vals)
# 	return width, height, theta
#
# # Function to interpolate a 3D position trajectory
# # Input:
# # - row: list of position
# # - res: number of interpolation for all the trajectory
# # - method: method use for the interpolation
# # Output:
# # - arr: arr of 1x3 with the 3D position
# def sample3DCurves(row, res=10, method='linear'):
# 	# edit: cleaner algebra
# 	x, *y, z = row
# 	# vecs between subsequently measured points
# 	vecs = np.diff(row)
# 	# path: cum distance along points (norm from first to ith point)
# 	path = np.cumsum(np.linalg.norm(vecs, axis=0))
# 	path = np.insert(path, 0, 0)
# 	## coords of interpolation
# 	coords = np.linspace(path[0], path[-1], res) #p[0]=0 p[-1]=max(p)
# 	# interpolation func for each axis with the path
# 	sampleX = interpolate.interp1d(path, row[0], kind=method)
# 	sampleY = interpolate.interp1d(path, row[1], kind=method)
# 	sampleZ = interpolate.interp1d(path, row[2], kind=method)
# 	# sample each dim
# 	xnew = sampleX(coords)
# 	ynew = sampleY(coords)
# 	znew = sampleZ(coords)
# 	arr = np.array([xnew, ynew, znew])
# 	return arr
#
# # Function to compute the density function of a Gaussian
# # Input:
# # - x: array of x axis data
# # - mean: mean of the Gaussian function
# # - sd: standard deviation of the Gaussian function
# # Output:
# # - prob_density: array with value of the Gaussian function according to the x array
# def normal_dist(x , mean , sd):
# 	prob_density = (1/(2*np.pi*sd**2) ) * np.exp(-0.5*((x-mean)/sd)**2)
# 	return prob_density
#
# ###################################################################################################
# ###################################################################################################
# # Function for GPS data processing
#
# # Function to find the GPS data according to a timestamp given
# # Input:
# # - gps_list: list of GPS position, array of 1x4, [0] timestamp
# # - index_list: list of index of the GPS list to test
# # - time_interval: timestamp given (s)
# # - limit_search: threshold for the time research (s)
# # Output:
# # - index: index corresponding to the closest timestamp found, -1 if no one found
# def research_index_for_time_gps(gps_list, index_list, time_interval, limit_search):
# 	result = 0
# 	diff = limit_search
# 	index = 0
# 	found_one = 0
# 	for i in index_list:
# 		if(abs(time_interval-gps_list[i][0])< limit_search and diff > abs(time_interval-gps_list[i][0])):
# 			diff = abs(time_interval-gps_list[i][0])
# 			result = gps_list[i][0]
# 			index = i
# 			found_one = 1
# 	if(found_one == 0):
# 		index = -1
# 	return index
#
# # Function to compute the speed of the GPS (m/s)
# # Input:
# # - GPS_front_utm_data: list of GPS position, array of 1x4, [0] timestamp, [1] x(m), [2] y(m), [3] z(m)
# # Output:
# # - linear_speed_gps_utm: list of the GPS speed
# def speed_gps(GPS_front_utm_data):
# 	linear_speed_gps_utm = []
# 	for i in range(0, len(GPS_front_utm_data)):
# 		if(i<len(GPS_front_utm_data)-1):
# 			data_i = GPS_front_utm_data[i]
# 			data_i2 = GPS_front_utm_data[i+1]
# 			distance = np.linalg.norm(np.array([data_i2[1],data_i2[2],data_i2[3]])-np.array([data_i[1],data_i[2],data_i[3]]))
# 			time_diff = data_i2[0]-data_i[0]
# 			speed = distance/time_diff
# 			if(speed<10):
# 				linear_speed_gps_utm.append(np.array([data_i[0],distance/time_diff]))
# 	return linear_speed_gps_utm
#
# #########################################################################################################################
#
# def Rx(theta):
# 	return np.matrix([[1, 0, 0], [0, math.cos(theta), -math.sin(theta)], [0, math.sin(theta), math.cos(theta)]])
#
# def Ry(theta):
# 	return np.matrix([[math.cos(theta), 0, math.sin(theta)], [0, 1, 0], [-math.sin(theta), 0, math.cos(theta)]])
#
# def Rz(theta):
# 	return np.matrix([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0, 0, 1]])
#
# def traj_simulation(t, speed_linear, speed_yaw, speed_pitch, speed_roll):
# 	# Init
# 	T_3d = []
# 	yaw = []
# 	pitch = []
# 	roll = []
# 	x = []
# 	y = []
# 	z = []
# 	T_3d.append(np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
# 	yaw.append(0)
# 	pitch.append(0)
# 	roll.append(0)
# 	x.append(0)
# 	y.append(0)
# 	z.append(0)
# 	dt = (t[1]-t[0])
#
# 	for i in range(1, len(t)):
# 		# Update coordinates
# 		dx = speed_linear[i] * dt
# 		dy = 0
# 		dz = 0
# 		d_yaw = speed_yaw[i] * dt
# 		d_pitch = speed_pitch[i] * dt
# 		d_roll = speed_roll[i] * dt
#
# 		former_yaw = yaw[i - 1]
# 		new_yaw = former_yaw + speed_yaw[i] * dt
# 		former_pitch = pitch[i - 1]
# 		new_pitch = former_pitch + speed_pitch[i] * dt
# 		former_roll = roll[i - 1]
# 		new_roll = former_roll + speed_roll[i] * dt
#
# 		# Compute new pose
# 		R_transition_3D = np.array(Rz(d_yaw) @ Ry(d_pitch) @ Rx(d_roll))
# 		Transition_3D = np.array([x[i - 1], y[i - 1], z[i - 1]]) + np.array([dx, dy, dz])
# 		P_new_3D = R_transition_3D[0:3, 0:3] @ Transition_3D
# 		R_new_3D = Rz(new_yaw) @ Ry(new_pitch) @ Rx(new_roll)
# 		T_new_3d = np.column_stack((R_new_3D, P_new_3D.T))
# 		T_new_new_3d = np.concatenate((T_new_3d, np.array([[0, 0, 0, 1]])), axis=0)
#
# 		# Save new 3D coordinates
# 		T_3d.append(T_new_new_3d)
# 		yaw.append(new_yaw)
# 		pitch.append(new_pitch)
# 		roll.append(new_roll)
# 		x.append(P_new_3D.flatten()[0])
# 		y.append(P_new_3D.flatten()[1])
# 		z.append(P_new_3D.flatten()[2])
#
# 	return T_3d, yaw, pitch, roll, x, y, z
#
# def noise_apply(mode, mean_e, std_e, mean_g, std_g, mean_noise_t, std_noise_t, rate, T_arr, t, P1, P2, P3):
# 	P1_ref = []
# 	P2_ref = []
# 	P3_ref = []
# 	P1_wnoise_sub = []
# 	P2_wnoise_sub = []
# 	P3_wnoise_sub = []
# 	P1_noise = []
# 	P2_noise = []
# 	P3_noise = []
# 	P1_noise_sub = []
# 	P2_noise_sub = []
# 	P3_noise_sub = []
# 	t_ref_1 = []
# 	t_ref_2 = []
# 	t_ref_3 = []
# 	t_sub_1 = []
# 	t_sub_2 = []
# 	t_sub_3 = []
# 	e_noise = []
# 	t_noise_1 = []
# 	t_noise_2 = []
# 	t_noise_3 = []
#
# 	d1 = []
# 	d2 = []
# 	d3 = []
#
# 	global_noise_1 = np.random.normal(mean_g, std_g, size=3)
# 	global_noise_2 = np.random.normal(mean_g, std_g, size=3)
# 	global_noise_3 = np.random.normal(mean_g, std_g, size=3)
#
# 	# Noise measurements
# 	for i in range(0, len(T_arr)):
# 		# Reference
# 		p1_ref = (T_arr[i] @ P1[0:4, 3]).T
# 		p2_ref = (T_arr[i] @ P2[0:4, 3]).T
# 		p3_ref = (T_arr[i] @ P3[0:4, 3]).T
# 		d1.append(abs(np.linalg.norm(p1_ref - p2_ref))*1000)
# 		d2.append(abs(np.linalg.norm(p1_ref - p3_ref)) * 1000)
# 		d3.append(abs(np.linalg.norm(p2_ref - p3_ref)) * 1000)
#
# 		P1_ref.append(p1_ref)
# 		P2_ref.append(p2_ref)
# 		P3_ref.append(p3_ref)
#
# 		# Noise for each axis coordinate
# 		e_p1 = np.random.normal(mean_e, std_e, size=3)
# 		e_p2 = np.random.normal(mean_e, std_e, size=3)
# 		e_p3 = np.random.normal(mean_e, std_e, size=3)
# 		e_noise.append(np.array([e_p1, e_p2, e_p3, global_noise_1, global_noise_2, global_noise_3]))
#
# 		# Point with noise
# 		p1_noise = (T_arr[i] @ P1)[0:3, 3].T + e_p1 + global_noise_1
# 		p2_noise = (T_arr[i] @ P2)[0:3, 3].T + e_p2 + global_noise_2
# 		p3_noise = (T_arr[i] @ P3)[0:3, 3].T + e_p3 + global_noise_3
# 		P1_noise.append(p1_noise)
# 		P2_noise.append(p2_noise)
# 		P3_noise.append(p3_noise)
#
# 	# Check rate of T_arr
# 	diff_t = t[1] - t[0]
# 	subsampling_t = 1 / rate
# 	div = round(subsampling_t / diff_t)
#
# 	if (mode == "Sync"):
# 		for i in range(0, len(T_arr), div):
# 			temporal_noise = np.random.normal(mean_noise_t, std_noise_t, size=3)
# 			t_noise_1.append(temporal_noise[0])
# 			t_noise_2.append(temporal_noise[1])
# 			t_noise_3.append(temporal_noise[2])
# 			t_ref_1.append(t[i])
# 			t_ref_2.append(t[i])
# 			t_ref_3.append(t[i])
# 			t_sub_1.append(t[i]+temporal_noise[0])
# 			t_sub_2.append(t[i]+temporal_noise[0])
# 			t_sub_3.append(t[i]+temporal_noise[0])
# 			P1_noise_sub.append(P1_noise[i])
# 			P2_noise_sub.append(P2_noise[i])
# 			P3_noise_sub.append(P3_noise[i])
#
# 			p1_ref = (T_arr[i] @ P1)[0:3, 3].T
# 			p2_ref = (T_arr[i] @ P2)[0:3, 3].T
# 			p3_ref = (T_arr[i] @ P3)[0:3, 3].T
# 			P1_wnoise_sub.append(p1_ref)
# 			P2_wnoise_sub.append(p2_ref)
# 			P3_wnoise_sub.append(p3_ref)
#
# 	if (mode == "Async"):
# 		div_async = round(div / 3)
# 		sort_arr = np.array([1, 0, 0])
# 		for i in range(0, len(T_arr), div_async):
# 			temporal_noise = np.random.normal(mean_noise_t, std_noise_t, size=1)
# 			if (sort_arr[0] == 1 and sort_arr[1] == 0 and sort_arr[2] == 0):
# 				t_ref_1.append(t[i])
# 				t_noise_1.append(temporal_noise[0])
# 				t_sub_1.append(t[i]+temporal_noise[0])
# 				P1_noise_sub.append(P1_noise[i])
# 				sort_arr[0] = 0
# 				sort_arr[1] = 1
# 				p1_ref = (T_arr[i] @ P1)[0:3, 3].T
# 				P1_wnoise_sub.append(p1_ref)
# 			elif (sort_arr[0] == 0 and sort_arr[1] == 1 and sort_arr[2] == 0):
# 				t_ref_2.append(t[i])
# 				t_noise_2.append(temporal_noise[0])
# 				t_sub_2.append(t[i]+temporal_noise[0])
# 				P2_noise_sub.append(P2_noise[i])
# 				sort_arr[1] = 0
# 				sort_arr[2] = 1
# 				p2_ref = (T_arr[i] @ P2)[0:3, 3].T
# 				P2_wnoise_sub.append(p2_ref)
# 			elif (sort_arr[0] == 0 and sort_arr[1] == 0 and sort_arr[2] == 1):
# 				t_ref_3.append(t[i])
# 				t_noise_3.append(temporal_noise[0])
# 				t_sub_3.append(t[i]+temporal_noise[0])
# 				P3_noise_sub.append(P3_noise[i])
# 				sort_arr[2] = 0
# 				sort_arr[0] = 1
# 				p3_ref = (T_arr[i] @ P3)[0:3, 3].T
# 				P3_wnoise_sub.append(p3_ref)
#
# 	return P1_ref, P2_ref ,P3_ref, P1_noise, P2_noise, P3_noise, P1_noise_sub, P2_noise_sub , P3_noise_sub, t_sub_1, t_sub_2, t_sub_3, e_noise, t_noise_1, t_noise_2, t_noise_3, t_ref_1, t_ref_2, t_ref_3, P1_wnoise_sub, P2_wnoise_sub, P3_wnoise_sub, d1, d2, d3

def thresold_raw_data(time, distance, azimuth, elevation, e_distance, e_azimuth, e_elevation, time_limit):
	begin_t = 0
	index_time = []
	for i in range(0, len(time) - 1):
		if (abs(time[i + 1] - time[i]) > time_limit):
			if (abs((i - 1) - begin_t >= 2)):
				index_time.append([begin_t, i - 1])
				begin_t = i
		if ((abs(time[i + 1] - time[i]) <= time_limit) and (
				i + 1 == len(time) - 1)):
			index_time.append([begin_t, i + 1])

	index_final = []
	for i in index_time:
		for j in range(i[0], i[1] - 1):
			if (time[j + 1] != time[j]):
				deltad = abs(distance[j + 1] - distance[j])
				deltae = abs(elevation[j + 1] - elevation[j])
				deltaa = abs(azimuth[j + 1] - azimuth[j])
				if (deltaa > 6):
					deltaa = abs(deltaa - 2*math.pi)

				if (deltad/abs(time[j + 1] - time[j]) < e_distance and
						deltae/abs(time[j + 1] - time[j]) < e_elevation and
						deltaa/abs(time[j + 1] - time[j]) < e_azimuth):
					index_final.append(j)
	return index_final


# def random_splitting(data: np.ndarray, threshold: float = 0.8):
# 	"""
# 	Randomly split a numpy array in two using a uniform distribution.
#
# 	Parameters
# 	----------
# 	data : numpy.ndarray
# 		The numpy array to be split.
# 	threshold : float
# 		The percentage used to split data. Default = 0.8
#
# 	Returns
# 	-------
# 	out: tuple of numpy array
# 		The first numpy array contains approximately the percentage of elements given by 'threshold' and
# 		the second numpy array contains the complement of the first.
# 	"""
# 	assert 0 < threshold <= 1, "The threshold must be greater than 0 and less than or equal to 1."
#
# 	prob = np.random.default_rng().uniform(size=data.shape[0])
# 	mask = prob <= threshold
#
# 	return data[mask], data[~mask]

def random_splitting_mask(data: np.ndarray, threshold: float = 0.75):
	"""
	Randomly split a numpy array in two using a uniform distribution.

	Parameters
	----------
	data : numpy.ndarray
		The numpy array to be split.
	threshold : float
		The percentage used to split data. Default = 0.8

	Returns
	-------
	out: mask numpy array
		The first numpy array contains approximately the percentage of elements given by 'threshold' and
		the second numpy array contains the complement of the first.
	"""
	assert 0 < threshold <= 1, "The threshold must be greater than 0 and less than or equal to 1."

	prob = np.random.default_rng().uniform(size=data.shape[0])
	mask = prob <= threshold

	return mask

# # Function which converts pose and roll,pitch,yaw to Transformation matrix
# # Input:
# # - file: name of the rosbag to open
# # Output:
# # - pose: list of 4x4 pose matrix
# # - time_icp: list of timestamp for each pose
# def tf_from_pose_roll_pitch_yaw(pose6dof):
#     x=pose6dof[0]
#     y=pose6dof[1]
#     z=pose6dof[2]
#     roll=pose6dof[3]
#     pitch=pose6dof[4]
#     yaw=pose6dof[5]
#     T = np.identity(4)
#     Rot_r = np.array([[np.cos(yaw), -np.sin(yaw), 0],
#                      [np.sin(yaw), np.cos(yaw), 0],
#                      [0, 0, 1]])
#     T[0:3,0:3]=Rot_r
#     T[0,3] = x
#     T[1,3] = y
#     T[2,3] = z
#     return T

def uniform_random_mask(size: int, threshold: float = 0.75):
	"""
	Return a mask filter using a uniform distribution.

	Parameters
	----------
	size : int
		The size of the mask to generate.
	threshold : float
		The threshold used to generate the mask. Default = 0.75

	Returns
	-------
	mask : ndarray
		A new 1D array holding boolean values.

	Examples
	--------
	>> a = np.arange(m*n).reshape(m,n)
	>> mask = uniform_random_mask(m.shape[0], 0.75) # filter along first axis
	>> filtered_m = m[mask]
	>> mask = uniform_random_mask(m.shape[1], 0.75) # filter along second axis
	>> filtered_m = m[:,mask]
	"""
	assert 0 < threshold <= 1, "The threshold must be greater than 0 and less than or equal to 1."

	uniform_dist = np.random.default_rng().uniform(size=size)
	mask = uniform_dist <= threshold

	return mask

def distance_time_trajectory(list_trajectories_split,trimble_1,time_trimble_1):
    distance = []
    time_travel = []

    for i in tqdm(list_trajectories_split):

        index_1 = np.array([i[0,0],i[1,0]])

        traj_1_x = np.atleast_2d(trimble_1[0, index_1[0]:index_1[1]+1]).T
        traj_1_y = np.atleast_2d(trimble_1[1, index_1[0]:index_1[1]+1]).T
        traj_1_z = np.atleast_2d(trimble_1[2, index_1[0]:index_1[1]+1]).T

        time_1 = time_trimble_1[index_1[0]:index_1[1]+1]

        dist = 0
        for j in range(0,len(traj_1_x)-1):
            point_before = np.array([traj_1_x[j],traj_1_y[j],traj_1_z[j]])
            point_after = np.array([traj_1_x[j+1],traj_1_y[j+1],traj_1_z[j+1]])
            diff = point_before - point_after
            norm = np.linalg.norm(diff)
            dist = dist + norm

        distance.append(dist)
        time_travel.append(abs(time_1[-1]-time_1[0]))
    return sum(distance), sum(time_travel)

def if_file_exist(path,option):
    if(exists(path+option)):
        return path
    else:
        return ""