import numpy as np
import random
import math
from numpy import linalg
from scripts import theodolite_utils as tu
from scripts.theodolite_values import *
from scripts.theodolite_plot_function import *

###################################################################################################
###################################################################################################
# Function to process markers

# Function to calculate and plot the histogram of the marker error
# Input:
# - number_markers: number of markers used for the calibration
# - trimble_1, trimble_2, trimble_3: list of theodolite data
# - save_fig: param if we want to save the figure
# - name_file: name of the file to save the figure
# - scale_x_min, scale_x_max: value min and max for the histogram scale x axis
def markers_histogram(number_markers,trimble_1,trimble_2,trimble_3, save_fig, name_file, scale_x_min, scale_x_max):
	distance_error = []
	for i in range(0,number_markers):
		distance_error.append(np.linalg.norm(trimble_1[0:3,i]-trimble_2[0:3,i])*1000)
		distance_error.append(np.linalg.norm(trimble_3[0:3,i]-trimble_2[0:3,i])*1000)
		distance_error.append(np.linalg.norm(trimble_1[0:3,i]-trimble_3[0:3,i])*1000)
	plot_histogram(distance_error, save_fig, name_file, scale_x_min, scale_x_max, True)

###################################################################################################
###################################################################################################
# Function to process theodolite data

# Function to calculate the inter-prism distance for not moving time
# Input:
# - trimble_1, trimble_2, trimble_3: list of theodolite data
# - time_trimble_1, time_trimble_2, time_trimble_3: list of timestamp of theodolite data
# - dist_max: distance max between two data (m)
# - time_max: time max between two data (s)
# - one_prism: option to only sort data coming from one prism, True: one prism, False: three prisms
# Output:
# - distance_error_12, distance_error_13, distance_error_23: list of inter-prism distance
def error_distance_not_moving(trimble_1, trimble_2, trimble_3, time_trimble_1, time_trimble_2, time_trimble_3, dist_max, time_max, one_prism):
	# find tuple of not moving point
	tuple_not_moving_trimble_1, tuple_not_moving_trimble_2, tuple_not_moving_trimble_3 = cluster_not_moving_points(trimble_1, trimble_2, trimble_3, time_trimble_1, time_trimble_2, time_trimble_3, dist_max, time_max)
	mean_position_static_pose_1 = []
	mean_position_static_pose_2 = []
	mean_position_static_pose_3 = []
	tuple_mean_position = []
	# If one prism
	if(one_prism):
		for i in tuple_not_moving_trimble_1:
			mean_position_static_pose_1.append(np.mean(trimble_1[0:3,i],axis=1))
		for i in tuple_not_moving_trimble_2:
			mean_position_static_pose_2.append(np.mean(trimble_2[0:3,i],axis=1))
		for i in tuple_not_moving_trimble_3:
			mean_position_static_pose_3.append(np.mean(trimble_3[0:3,i],axis=1))

		for i in mean_position_static_pose_1:
			bool_1 = False
			bool_2 = False
			list_temporary = []
			comparaison_ij = 10
			comparaison_ik = 10
			point_ij = np.array([10000,0,0])
			point_ik = np.array([10000,0,0])
			list_temporary.append(i)
			for j in mean_position_static_pose_2:
				distance_ij = np.linalg.norm(i-j)
				if(distance_ij < 0.1 and comparaison_ij > distance_ij):
					comparaison_ij = distance_ij
					point_ij = j
					bool_1 = True
			if(bool_1 == True):
				list_temporary.append(point_ij)
				for k in mean_position_static_pose_3:
					distance_ik = np.linalg.norm(i-k)
					if(np.linalg.norm(i-k) < 0.1 and comparaison_ik > distance_ik):
						comparaison_ik = distance_ik
						point_ik = k
						bool_2 = True
			if(bool_2 == True):
				list_temporary.append(point_ik)
				tuple_mean_position.append(list_temporary)
	else:
		# If more 3 prisms
		for i in tuple_not_moving_trimble_1:
			mean_position_static_pose_1.append(np.mean(trimble_1[0:3,i],axis=1))
		for i in tuple_not_moving_trimble_2:
			mean_position_static_pose_2.append(np.mean(trimble_2[0:3,i],axis=1))
		for i in tuple_not_moving_trimble_3:
			mean_position_static_pose_3.append(np.mean(trimble_3[0:3,i],axis=1))

		for i in mean_position_static_pose_1:
			bool_1 = False
			bool_2 = False
			list_temporary = []
			comparaison_ij = 10
			comparaison_ik = 10
			point_ij = np.array([10000,0,0])
			point_ik = np.array([10000,0,0])
			list_temporary.append(i)
			for j in mean_position_static_pose_2:
				distance_ij = np.linalg.norm(i-j)
				if(distance_ij < 2 and comparaison_ij > distance_ij):
					comparaison_ij = distance_ij
					point_ij = j
					bool_1 = True
			if(bool_1 == True):
				list_temporary.append(point_ij)
				for k in mean_position_static_pose_3:
					distance_ik = np.linalg.norm(i-k)
					if(np.linalg.norm(i-k) < 2 and comparaison_ik > distance_ik):
						comparaison_ik = distance_ik
						point_ik = k
						bool_2 = True
			if(bool_2 == True):
				list_temporary.append(point_ik)
				tuple_mean_position.append(list_temporary)

	if(one_prism == True):
		distance_error = []
		for i in tuple_mean_position:
			distance_error.append(np.linalg.norm(i[0]-i[1])*1000)
			distance_error.append(np.linalg.norm(i[2]-i[1])*1000)
			distance_error.append(np.linalg.norm(i[0]-i[2])*1000)
		return distance_error
	else:
		distance_error_12 = []
		distance_error_13 = []
		distance_error_23 = []
		print(np.linalg.norm(Prism_1-Prism_2))
		for i in tuple_mean_position:
			print(np.linalg.norm(i[0]-i[1]))
			distance_error_12.append(abs(np.linalg.norm(i[0]-i[1])-np.linalg.norm(Prism_1-Prism_2))*1000)
			distance_error_13.append(abs(np.linalg.norm(i[2]-i[1])-np.linalg.norm(Prism_3-Prism_2))*1000)
			distance_error_23.append(abs(np.linalg.norm(i[0]-i[2])-np.linalg.norm(Prism_1-Prism_3))*1000)
		return distance_error_12, distance_error_13, distance_error_23

# Function to slipt the data into different interval according to the timestamps of the measurement
# Input:
# - time_trimble_1, time_trimble_2, time_trimble_3: list of timestamp of theodolite data
# - limit_time_interval: distance max between two data (m)
# Output:
# - list_interval: list of interval for each prism
# - list_time: list of timestamp for each of these intervals
def split_time_interval_all_data(time_trimble_1, time_trimble_2, time_trimble_3, limit_time_interval):
	list_time_interval_1 = split_time_interval(time_trimble_1, limit_time_interval)
	list_time_interval_2 = split_time_interval(time_trimble_2, limit_time_interval)
	list_time_interval_3 = split_time_interval(time_trimble_3, limit_time_interval)
	list_interval = []
	list_interval.append(list_time_interval_1)
	list_interval.append(list_time_interval_2)
	list_interval.append(list_time_interval_3)
	list_time = []
	list_time.append(time_trimble_1)
	list_time.append(time_trimble_2)
	list_time.append(time_trimble_3)
	return list_interval, list_time

# Function to keep only the intervals where the three theodlites have done some measurements
# Input:
# - list_interval: list of interval for each prism
# - list_time: list of timestamp for each of these intervals
# - time_trimble_1, time_trimble_2, time_trimble_3: list of timestamp of theodolite data
# - limit_search: time max between two data (s)
# Output:
# - list_trajectories_split: list of the different time interval which contain three valid prism measurements
def merge_interval(list_interval, list_time, time_trimble_1, time_trimble_2, time_trimble_3, limit_search):
	ite_int = np.array([0,0,0])
	max_iterator = np.array([len(list_interval[0]),len(list_interval[1]),len(list_interval[2])])
	list_trajectories_split = []

	while(ite_int[0]<max_iterator[0] and ite_int[1]<max_iterator[1] and ite_int[2]<max_iterator[2]):
		Begin = np.array([list_interval[0][ite_int[0]][0],list_interval[1][ite_int[1]][0],list_interval[2][ite_int[2]][0]])
		End = np.array([list_interval[0][ite_int[0]][1],list_interval[1][ite_int[1]][1],list_interval[2][ite_int[2]][1]])
		max_Init = max(list_time[0][Begin[0]],list_time[1][Begin[1]],list_time[2][Begin[2]])
		max_trimble = np.where(np.array([list_time[0][Begin[0]],list_time[1][Begin[1]],list_time[2][Begin[2]]])==np.amax(np.array([list_time[0][Begin[0]],list_time[1][Begin[1]],list_time[2][Begin[2]]])))[0][0]
		min_End = min(list_time[0][End[0]],list_time[1][End[1]],list_time[2][End[2]])
		min_trimble = np.where(np.array([list_time[0][End[0]],list_time[1][End[1]],list_time[2][End[2]]]) == np.amin(np.array([list_time[0][End[0]],list_time[1][End[1]],list_time[2][End[2]]])))[0][0]

		if(min_End-max_Init<=0):
			ite_int[min_trimble]=ite_int[min_trimble]+1
		else:
			time_interval = np.array([max_Init,min_End])
			#print(time_interval)
			index_ini_1 = research_index_for_time(time_trimble_1, time_interval[0], limit_search)
			index_ini_2 = research_index_for_time(time_trimble_2, time_interval[0], limit_search)
			index_ini_3 = research_index_for_time(time_trimble_3, time_interval[0], limit_search)
			index_end_1 = research_index_for_time(time_trimble_1, time_interval[1], limit_search)
			index_end_2 = research_index_for_time(time_trimble_2, time_interval[1], limit_search)
			index_end_3 = research_index_for_time(time_trimble_3, time_interval[1], limit_search)
			#print(index_ini_1, index_ini_2, index_ini_3, index_end_1, index_end_2, index_end_3)

			if(index_ini_1!=-1 and index_ini_2!=-1 and index_ini_3!=-1 and index_end_1!=-1 and index_end_2!=-1 and index_end_3!=-1):
				if(index_end_1-index_ini_1>2 and index_end_2-index_ini_2>2 and index_end_3-index_ini_3>2):
					array_index = np.array([[index_ini_1,index_ini_2,index_ini_3],[index_end_1,index_end_2,index_end_3]])
					list_trajectories_split.append(array_index)
			ite_int[min_trimble]=ite_int[min_trimble]+1
	return list_trajectories_split

# Function to interpolate linearly all of these sub-trajectories for 3 prisms according to time
# Input:
# - time_step: list of interval for each prism
# - list_trajectories_split: list of the different time interval which contain three valid prism measurements
# - trimble_1, trimble_2, trimble_3: list of theodolite data
# - time_trimble_1, time_trimble_2, time_trimble_3: list of timestamp of theodolite data
# Output:
# - interpolated_time: list of time for each interpolated interval
# - interpolated_trajectories: list of interpolated trajectories
def time_interpolation_subtrajectories(time_step, list_trajectories_split, trimble_1, trimble_2, trimble_3, time_trimble_1, time_trimble_2, time_trimble_3):
	interpolated_trajectories = []
	interpolated_time = []
	for i in list_trajectories_split:
		# Compute limit of time interpolation
		debut_1 = i[:,0]
		debut_2 = i[:,1]
		debut_3 = i[:,2]
		traj1 = trimble_1[0:3,debut_1[0]:debut_1[1]+1]
		traj2 = trimble_2[0:3,debut_2[0]:debut_2[1]+1]
		traj3 = trimble_3[0:3,debut_3[0]:debut_3[1]+1]
		time1 = time_trimble_1[debut_1[0]:debut_1[1]+1]
		time2 = time_trimble_2[debut_2[0]:debut_2[1]+1]
		time3 = time_trimble_3[debut_3[0]:debut_3[1]+1]
		time_debut = max(time1[0],time2[0],time3[0])
		time_fin = min(time1[-1],time2[-1],time3[-1])
		# Compute the time interpolation which will be used
		diff_time = time_fin - time_debut
		time_resolution = int(diff_time/time_step)
		time_interpolated = np.linspace(time_debut, time_fin, num=time_resolution)
		# Interpolation
		f1_x = interpolate.interp1d(time1, traj1[0])
		f1_y = interpolate.interp1d(time1, traj1[1])
		f1_z = interpolate.interp1d(time1, traj1[2])
		traj1_x = f1_x(time_interpolated)
		traj1_y = f1_y(time_interpolated)
		traj1_z = f1_z(time_interpolated)
		f2_x = interpolate.interp1d(time2, traj2[0])
		f2_y = interpolate.interp1d(time2, traj2[1])
		f2_z = interpolate.interp1d(time2, traj2[2])
		traj2_x = f2_x(time_interpolated)
		traj2_y = f2_y(time_interpolated)
		traj2_z = f2_z(time_interpolated)
		f3_x = interpolate.interp1d(time3, traj3[0])
		f3_y = interpolate.interp1d(time3, traj3[1])
		f3_z = interpolate.interp1d(time3, traj3[2])
		traj3_x = f3_x(time_interpolated)
		traj3_y = f3_y(time_interpolated)
		traj3_z = f3_z(time_interpolated)
		# Store interpolation
		Interpolation_trimble_1 = np.array([traj1_x, traj1_y, traj1_z])
		Interpolation_trimble_2 = np.array([traj2_x, traj2_y, traj2_z])
		Interpolation_trimble_3 = np.array([traj3_x, traj3_y, traj3_z])
		# Keep time interpolation
		interpolated_time.append(time_interpolated)
		# Keep interpolated trajectories
		list_tem = []
		list_tem.append(Interpolation_trimble_1)
		list_tem.append(Interpolation_trimble_2)
		list_tem.append(Interpolation_trimble_3)
		interpolated_trajectories.append(list_tem)
	return interpolated_time, interpolated_trajectories

# Function to compute the point to point minimization with interpolated trajectories
# Input:
# - interpolated_time: list of time for each interpolated interval
# - interpolated_trajectories: list of interpolated trajectories
# Output:
# - Pose_lidar: list of lidar pose
# - Prism_corrected: list of corrected prism positions
# - list_lidar_time: list of lidar timestamp
def ptp_minimization_with_interpolated_trajectories(interpolated_trajectories, interpolated_time):
	# Prism positions measured by one theodolite
	P = np.array([[-0.12413052, -1.05061716, -0.30736107],[-0.32998385, -0.00439399,  0.32429536],
              [ 0.34745009,  0.2448696,  0.30031949],[ 1.,          1.,          1.        ]])
	# Doing a minimization between these not moving points, and the 3D prism coordinates
	# Pose_lidar is a list of each rigid transform founded
	list_lidar_time = []
	Pose_lidar = []
	Prism_corrected = []
	itera = 0
	for j in interpolated_trajectories:
		for i in range(0,len(j[0].T)):
			Q = np.array([j[0][:,i], j[1][:,i], j[2][:,i]]).T
			Q =np.concatenate((Q, np.array([[1,1,1]])), axis=0)
			T = point_to_point_minimization(P, Q)
			Pose_lidar.append(T)
			prism_correct = T@P
			Prism_corrected.append(prism_correct)
			list_lidar_time.append(interpolated_time[itera][i])
		itera = itera+1
	return Pose_lidar, Prism_corrected, list_lidar_time

# Function to find not moving time interval of warthog according its speed
# Input:
# - speed: list of 1x2 array of the speed, [0]: timestamp, [1]: linear speed
# - speed_limit: threshold for the speed to detect static pose
# - time_limit: minimum time of a static interval (s)
# Output:
# - not_moving_time: list of 1x2 array for each interval found, [0]: begin and [1]: end
def find_warthog_not_moving_time_interval(speed, speed_limit, time_limit):
	not_moving_time = []
	speed_low_detected = False
	begin_speed_low = 0
	for i in speed:
		  if(abs(i[1])<speed_limit and speed_low_detected==False):
		      begin_speed_low = i[0]
		      speed_low_detected = True
		  if(abs(i[1])>=speed_limit and speed_low_detected==True):
		      if(abs(begin_speed_low-i[0])>time_limit):
		          not_moving_time.append(np.array([begin_speed_low,i[0]]))
		      speed_low_detected = False
	if(speed_low_detected==True and abs(begin_speed_low-i[0])>time_limit):
		  not_moving_time.append(np.array([begin_speed_low,i[0]]))
	return not_moving_time

# Function to find not moving prism time
# Input:
# - not_moving_time: list of 1x2 array for each interval found, [0]: begin and [1]: end
# - time_trimble_1, time_trimble_2, time_trimble_3: list of timestamp of theodolite data
# - number_points_limit: minimum of prism positions in a static interval
# Output:
# - not_moving_prism_1, not_moving_prism_2, not_moving_prism_3: list of timestamp of each not moving prisms in the different intervals
def find_prism_not_moving_time(not_moving_time, time_trimble_1, time_trimble_2, time_trimble_3, number_points_limit):
	not_moving_prism_1 = []
	not_moving_prism_2 = []
	not_moving_prism_3 = []
	number_points_limit = 0
	for i in not_moving_time:
		  debut = i[0]
		  fin = i[1]
		  list_1 = []
		  list_2 = []
		  list_3 = []
		  for j1 in range(0,len(time_trimble_1)):
		      if(time_trimble_1[j1]>= debut and time_trimble_1[j1]<fin):
		          list_1.append(j1)
		  for j2 in range(0,len(time_trimble_2)):
		      if(time_trimble_2[j2]>= debut and time_trimble_2[j2]<fin):
		          list_2.append(j2)
		  for j3 in range(0,len(time_trimble_3)):
		      if(time_trimble_3[j3]>= debut and time_trimble_3[j3]<fin):
		          list_3.append(j3)
		  if(len(list_1)>number_points_limit and len(list_2)>number_points_limit and len(list_3)>number_points_limit):
		      not_moving_prism_1.append(list_1)
		      not_moving_prism_2.append(list_2)
		      not_moving_prism_3.append(list_3)
	return not_moving_prism_1, not_moving_prism_2, not_moving_prism_3

# Function to make tupple of prism positions when warthog is static
# Input:
# - not_moving_prism_1, not_moving_prism_2, not_moving_prism_3: list of timestamp of each not moving prisms in the different intervals
# - time_trimble_1, time_trimble_2, time_trimble_3: list of timestamp of theodolite data
# - time_limit: maximum time between timestamp to consider them close enough
# Output:
# - not_moving_tuple: list of prism tupple for each not moving interval
def make_tupple_not_moving_point(not_moving_prism_1, not_moving_prism_2, not_moving_prism_3, time_trimble_1, time_trimble_2, time_trimble_3, time_limit):
	not_moving_tuple = []
	for i,j,k in zip(not_moving_prism_1,not_moving_prism_2,not_moving_prism_3):
		if(len(i)>0 and len(j)>0 and len(k)>0):
			for l1 in i:
				for l2 in j:
					for l3 in k:
						if(abs(time_trimble_1[l1]-time_trimble_2[l2])<time_limit and abs(time_trimble_1[l1]-time_trimble_3[l3])<time_limit and abs(time_trimble_2[l2]-time_trimble_3[l3])<time_limit):
							not_moving_tuple.append(np.array([l1,l2,l3]))
	return not_moving_tuple

# Function to compute the distance with tupple of points
# Input:
# - trimble_1, trimble_2, trimble_3: list of theodolite data
# - not_moving_tuple: list of prism tupple for each not moving interval
# Output:
# - distance_12, distance_13, distance_23: list of inter-prism distance (mm)
def compute_distance(trimble_1, trimble_2, trimble_3, not_moving_tuple):
	distance_12 = []
	distance_13 = []
	distance_23 = []
	for i in not_moving_tuple:
		position_1 = trimble_1[0:3,i[0]]
		position_2 = trimble_2[0:3,i[1]]
		position_3 = trimble_3[0:3,i[2]]

		distance_12.append(abs(np.linalg.norm(position_1-position_2))*1000)
		distance_13.append(abs(np.linalg.norm(position_1-position_3))*1000)
		distance_23.append(abs(np.linalg.norm(position_2-position_3))*1000)
	return distance_12, distance_13, distance_23

# Function to find not moving prism time after interpolation
# Input:
# - not_moving_time: list of not moving time interval
# - interpolated_trajectories: list of interpolated trajectories
# - interpolated_time: list of timestamp for each interpolated trajectories
# Output:
# - not_moving_interpolated_prism: list of not moving interpolated points
def find_interpolated_prism_not_moving_time(not_moving_time, interpolated_trajectories, interpolated_time):
	not_moving_interpolated_prism = []
	for j,k in zip(interpolated_trajectories, interpolated_time):
		list_pose = []
		for i in not_moving_time:
			debut = i[0]
			fin = i[1]
			for j1 in range(0,len(k)):
				if(k[j1]>= debut and k[j1]<fin):
					  list_pose.append(j1)
		not_moving_interpolated_prism.append(list_pose)
	return not_moving_interpolated_prism

# Function to compute distance with tupple of interpolated points
# Input:
# - not_moving_interpolated_prism: list of not moving interpolated points
# - interpolated_trajectories: list of interpolated trajectories
# Output:
# - distance_12, distance_13, distance_23: list of inter-prism distance (mm)
def compute_interpolated_distance(not_moving_interpolated_prism, interpolated_trajectories):
	distance_12 = []
	distance_13 = []
	distance_23 = []
	for i,j in zip(not_moving_interpolated_prism,interpolated_trajectories):
		for k in i:
			position_1 = j[0][0:3,k]
			position_2 = j[1][0:3,k]
			position_3 = j[2][0:3,k]

			distance_12.append(abs(np.linalg.norm(position_1-position_2))*1000)
			distance_13.append(abs(np.linalg.norm(position_1-position_3))*1000)
			distance_23.append(abs(np.linalg.norm(position_2-position_3))*1000)
	return distance_12, distance_13, distance_23

# Function to compute distance with tupple of interpolated points
# Input:
# - speed_min: minimum speed (m/s)
# - speed_max: maximum speed (m/s)
# - speed_step: speed step for interval (m/s)
# - interpolated_trajectories: list of interpolated trajectories
# - interpolated_time: list of timestamp for each interpolated trajectories
# - speed: list of 1x2 array of the robot speed, [0]: timestamp (s) and [1]: linear speed (m/s)
# Output:
# - distance_12_speed, distance_13_speed, distance_23_speed: list of the inter-prism distance for each of the speed interval (mm)
# - speed_interval: list of the speed intervals computed (m/s)
def sort_interpolated_point_accoridng_to_speed(speed_min, speed_max, speed_step, interpolated_trajectories, interpolated_time, speed):
	speed_value = np.linspace(speed_min,speed_max,speed_step)/100
	speed_interval = []
	distance_12_speed = []
	distance_13_speed = []
	distance_23_speed = []
	for i in range(0,len(speed_value)):
		if(i!=len(speed_value)-1):
			speed_interval.append(np.array([speed_value[i],speed_value[i+1]]))
			distance_12_speed.append([])
			distance_13_speed.append([])
			distance_23_speed.append([])
	for i,j in zip(interpolated_trajectories, interpolated_time):
		for k in range(0,len(j)):
			index = research_index_for_time_speed(speed, j[k], 0.1)
			if(index!=-1):
				speed_point = abs(speed[index][1])
				position_1 = i[0][0:3,k]
				position_2 = i[1][0:3,k]
				position_3 = i[2][0:3,k]
				dist_12 = abs(np.linalg.norm(position_1-position_2))*1000
				dist_13 = abs(np.linalg.norm(position_1-position_3))*1000
				dist_23 = abs(np.linalg.norm(position_2-position_3))*1000
				for l in range(0,len(speed_interval)):
					if(speed_interval[l][0]<=speed_point and speed_interval[l][1]>speed_point):
						distance_12_speed[l].append(dist_12)
						distance_13_speed[l].append(dist_13)
						distance_23_speed[l].append(dist_23)
						break
	return distance_12_speed, distance_13_speed, distance_23_speed, speed_interval

# Function to compute distance with tupple of interpolated points
# Input:
# - distance_12_not_moving, distance_13_not_moving, distance_23_not_moving: list of inter-prism distance computed for not moving prisms (mm)
# - Dist_prism_12, Dist_prism_13, Dist_prism_23: distance between prisms (mm)
# - speed_interval: list of the speed intervals computed (m/s)
# - distance_12_speed, distance_13_speed, distance_23_speed: list of the inter-prism distance for each of the speed interval (mm)
# Output:
# - speed_axis: list of x axis linear speed (m/s)
# - mean_list: list of mean of distance found for each speed axis (mm)
# - std_list: list of standard deviation of distance found for each speed axis (mm)
# - true_value_list: list of true distance (mm)
def statistic_dynamic_interpolated_points(distance_12_not_moving, distance_13_not_moving, distance_23_not_moving, Dist_prism_12, Dist_prism_13, Dist_prism_23, speed_interval, distance_12_speed, distance_13_speed, distance_23_speed):
	mean_list = []
	std_list = []
	true_value_list = []
	mean_12 = []
	mean_13 = []
	mean_23 = []
	std_12 = []
	std_13 = []
	std_23 = []
	true_value_12 = []
	true_value_13 = []
	true_value_23 = []
	speed_axis = []
	speed_axis.append(0)
	mean_12.append(np.mean(distance_12_not_moving))
	mean_13.append(np.mean(distance_13_not_moving))
	mean_23.append(np.mean(distance_23_not_moving))
	std_12.append(np.std(distance_12_not_moving))
	std_13.append(np.std(distance_13_not_moving))
	std_23.append(np.std(distance_23_not_moving))
	true_value_12.append(Dist_prism_12)
	true_value_13.append(Dist_prism_13)
	true_value_23.append(Dist_prism_23)
	for i,j,k,l in zip(speed_interval, distance_12_speed, distance_13_speed, distance_23_speed):
		speed_axis.append((i[0]+i[1])/2)
		mean_12.append(np.mean(j))
		mean_13.append(np.mean(k))
		mean_23.append(np.mean(l))
		std_12.append(np.std(j))
		std_13.append(np.std(k))
		std_23.append(np.std(l))
		true_value_12.append(Dist_prism_12)
		true_value_13.append(Dist_prism_13)
		true_value_23.append(Dist_prism_23)
	mean_list.append(mean_12)
	mean_list.append(mean_13)
	mean_list.append(mean_23)
	std_list.append(std_12)
	std_list.append(std_13)
	std_list.append(std_23)
	true_value_list.append(true_value_12)
	true_value_list.append(true_value_13)
	true_value_list.append(true_value_23)
	return speed_axis, mean_list, std_list, true_value_list

###################################################################################################
###################################################################################################

# Function to process theodolite data in once
# Input:
# - file_marker: file of the markers
# - file_rosbag_theodolite: rosbag of theodolite data
# - file_rosbag_imu: rosbag of imu data
# Output:
# - interpolated_trajectories: list of interpolated trajectories
# - interpolated_time: list of timestamp for each interpolated trajectories
# - speed: list of 1x2 array of the robot speed, [0]: timestamp (s) and [1]: linear speed (m/s)
# - angular_speed: list of 1x2 array of the robot angular speed around Z axis, [0]: timestamp (s) and [1]: angular speed (deg/s)
# - accel: list of 1x2 array of the robot acceleration, [0]: timestamp (s) and [1]: acceleration (m/s^2)
def process_data_theodolite(file_marker, file_rosbag_theodolite, file_rosbag_imu):
	print("Marker data reading")
	# Markers reference
	marker_1, marker_2, marker_3, T_1, T_2, T_3 = read_marker_file(file_marker, 1)
	Tf = []
	Tf.append(T_1)
	Tf.append(T_2)
	Tf.append(T_3)
	print("Theodolite data reading")
	# Theodolite data reading
	trajectory_trimble_1, trajectory_trimble_2, trajectory_trimble_3, time_trimble_1, time_trimble_2, time_trimble_3 = read_rosbag_theodolite_with_tf(file_rosbag_theodolite, Tf)
	trimble_1 = np.array(trajectory_trimble_1).T
	trimble_2 = np.array(trajectory_trimble_2).T
	trimble_3 = np.array(trajectory_trimble_3).T

	print("Process theodolite trajectories")
	# Slipt the data into different interval according to the timestamps of the measurments
	limit_time_interval = 3
	list_interval, list_time = split_time_interval_all_data(time_trimble_1, time_trimble_2, time_trimble_3, limit_time_interval)
	# Keep only the intervals where the three theodlites have done some measurements at the same time
	limit_search = 2
	list_trajectories_split = merge_interval(list_interval, list_time, time_trimble_1, time_trimble_2, time_trimble_3, limit_search)
	# Interpolate linearly all of these sub-trajectories
	time_step = 0.05
	interpolated_time, interpolated_trajectories = time_interpolation_subtrajectories(time_step, list_trajectories_split, trimble_1, trimble_2, trimble_3, time_trimble_1, time_trimble_2, time_trimble_3)
	# Ptp minimization with interpolated trajectories
	Pose_lidar, Prism_corrected, list_lidar_time = ptp_minimization_with_interpolated_trajectories(interpolated_trajectories, interpolated_time)

	print("Linear velocity reading")
	if(file_rosbag_imu=="/home/maxime/theodolites_tests/data_for_processing/quarry/2020-10-08-13-35-15_start_stop_mapping_but_sensors_missing_imu_data.bag"):
		wheel = True
	else:
		wheel = False
	# Read rosbag for linear velocity
	speed, accel = read_rosbag_imu_node(file_rosbag_imu, wheel)
	print("Angular velocity reading")
	# Read rosbag for angular velocity around Z
	angular_speed = read_rosbag_imu_data(file_rosbag_imu, wheel)
	print("Finish !")

	return interpolated_trajectories, interpolated_time, speed, angular_speed, accel

def process_data_TS(path, inter_error, file_rosbag_imu):

	prefix = "GP-10-20"
	P1 = np.array(read_prediction_data_csv_file(path + prefix + "_1.csv"))
	P2 = np.array(read_prediction_data_csv_file(path + prefix + "_2.csv"))
	P3 = np.array(read_prediction_data_csv_file(path + prefix + "_3.csv"))

	dist_prism = []
	origin = 0
	for i in range(0, len(P1[:, 0])):
		dp1 = abs(np.linalg.norm(P1[i, 1:4] - P2[i, 1:4]) - inter_error[0])*1000
		dp2 = abs(np.linalg.norm(P1[i, 1:4] - P3[i, 1:4]) - inter_error[1])*1000
		dp3 = abs(np.linalg.norm(P3[i, 1:4] - P2[i, 1:4]) - inter_error[2])*1000
		dist_prism.append(np.array([P1[i, 0] - origin, dp1, dp2, dp3]))
	dist_prism = np.array(dist_prism)

	print("Linear velocity reading")

	# Read rosbag for linear velocity
	speed, accel = read_rosbag_imu_node(file_rosbag_imu, True)
	print("Angular velocity reading")
	# Read rosbag for angular velocity around Z
	angular_speed = read_rosbag_imu_data(file_rosbag_imu, True)

	linear_speed_list = []
	angular_speed_list = []
	accel_list = []
	mean_error_prisms_list = []
	timestamp = []
	for i in dist_prism:
		index = research_index_for_time_speed(speed, i[0], 0.4)
		if (index != -1):
			speed_value = speed[index][1]
			index = research_index_for_time_speed(angular_speed, i[0], 0.4)
			if (index != -1):
				angular_value = angular_speed[index][1]
				index = research_index_for_time_speed(accel, i[0], 0.4)
				if (index != -1):
					accel_value = accel[index][1]
					dist_12 = i[1]
					dist_13 = i[2]
					dist_23 = i[3]
					timestamp.append(i[0])
					linear_speed_list.append(speed_value)
					angular_speed_list.append(angular_value)
					accel_list.append(accel_value)
					mean_error_prisms_list.append(dist_12)
					timestamp.append(i[0])
					linear_speed_list.append(speed_value)
					angular_speed_list.append(angular_value)
					accel_list.append(accel_value)
					mean_error_prisms_list.append(dist_13)
					timestamp.append(i[0])
					linear_speed_list.append(speed_value)
					angular_speed_list.append(angular_value)
					accel_list.append(accel_value)
					mean_error_prisms_list.append(dist_23)
					#mean_error_prisms_list.append(np.mean(np.array([dist_12, dist_13, dist_23])))
	print("Finish !")
	return timestamp, linear_speed_list, angular_speed_list, accel_list, mean_error_prisms_list

# Function to process theodolite data in once
# Input:
# - interpolated_trajectories: list of interpolated trajectories
# - interpolated_time: list of timestamp for each interpolated trajectories
# - speed: list of 1x2 array of the robot speed, [0]: timestamp (s) and [1]: linear speed (m/s)
# - angular_speed: list of 1x2 array of the robot angular speed around Z axis, [0]: timestamp (s) and [1]: angular speed (deg/s)
# - accel: list of 1x2 array of the robot acceleration, [0]: timestamp (s) and [1]: acceleration (m/s^2)
# - linear_speed_l: list of 1x2 array of the linear speed from different experiments, [0]: timestamp (s) and [1]: linear speed (m/s)
# - angular_speed_l: list of 1x2 array of the robot angular speed around Z axis from different experiments, [0]: timestamp (s) and [1]: angular speed (deg/s)
# - accel_l: list of 1x2 array of the robot acceleration from different experiments, [0]: timestamp (s) and [1]: acceleration (m/s^2)
# - mean_error_prisms_l: list of inter-prism error from different experiments (mm)
# Output:
# - linear_speed_list: list of 1x2 array of the linear speed from different experiments, [0]: timestamp (s) and [1]: linear speed (m/s)
# - angular_speed_list: list of 1x2 array of the robot angular speed around Z axis from different experiments, [0]: timestamp (s) and [1]: angular speed (deg/s)
# - accel_list: list of 1x2 array of the robot acceleration from different experiments, [0]: timestamp (s) and [1]: acceleration (m/s^2)
# - mean_error_prisms_list: list of inter-prism error from different experiments (mm)
def sort_interpolated_point_according_to_speed_and_angular(interpolated_trajectories, interpolated_time, speed, angular, accel, linear_speed_l, angular_speed_l, accel_l, mean_error_prisms_l):
	linear_speed_list = linear_speed_l
	angular_speed_list = angular_speed_l
	accel_list = accel_l
	mean_error_prisms_list = mean_error_prisms_l
	for i,j in zip(interpolated_trajectories, interpolated_time):
		for k in range(0,len(j)):
			index = research_index_for_time_speed(speed, j[k], 0.4)
			if(index!=-1):
				speed_value = speed[index][1]
				index = research_index_for_time_speed(angular, j[k], 0.4)
				if(index!=-1):
					angular_value = angular[index][1]
					index = research_index_for_time_speed(accel, j[k], 0.4)
					if(index!=-1):
						accel_value = accel[index][1]
						position_1 = i[0][0:3,k]
						position_2 = i[1][0:3,k]
						position_3 = i[2][0:3,k]
						dist_12 = abs(np.linalg.norm(position_1-position_2)*1000-Dist_prism_12)
						dist_13 = abs(np.linalg.norm(position_1-position_3)*1000-Dist_prism_13)
						dist_23 = abs(np.linalg.norm(position_2-position_3)*1000-Dist_prism_23)
						linear_speed_list.append(speed_value)
						angular_speed_list.append(angular_value)
						accel_list.append(accel_value)
						#mean_error_prisms_list.append(np.mean(np.array([dist_12, dist_13, dist_23])))
						mean_error_prisms_list.append(dist_12)
						mean_error_prisms_list.append(dist_13)
						mean_error_prisms_list.append(dist_23)
	return linear_speed_list, angular_speed_list, accel_list, mean_error_prisms_list

###################################################################################################
###################################################################################################

# Function to compute the propagation of the noise during ptp
# Input:
# - P: array of three points representing the position of the prisms
# - noise_min: miminum noise used (m)
# - noise_max: maximum noise used (m)
# - noise_step: step for the noise (m)
# - number: number of iteration of ptp to compute statistic
# Output:
# - noise: list of noise used
# - statistic_x, statistic_y, statistic_z: list of 1x2 array of the statistic on the X, Y and Z axis, [0]: mean (m) and [1]: standard deviation (m)
# - statistic_roll, statistic_pitch, statistic_yaw: list of 1x2 array of the statistic on the roll, picth and yaw, [0]: mean (deg) and [1]: standard deviation (deg)
def analyze_std_ptp(P, noise_min, noise_max, noise_step, number):
	noise = np.linspace(noise_min,noise_max, noise_step)/1000
	statistic_x = []
	statistic_y = []
	statistic_z = []
	statistic_roll = []
	statistic_pitch = []
	statistic_yaw = []
	for i in noise:
		list_x = []
		list_y = []
		list_z = []
		list_roll = []
		list_pitch = []
		list_yaw = []
		for j in range(0,number):
			noise_value = i
			theta, phi = np.random.uniform(0, 2*np.pi ,3), np.random.uniform(0, np.pi, 3)
			X = noise_value * np.sin(phi) * np.cos(theta)
			Y = noise_value * np.sin(phi) * np.sin(theta)
			Z = noise_value * np.cos(phi)
			noise_point = np.array([X,Y,Z])
			Q = np.array([P[0] + noise_point[:,0], P[1] + noise_point[:,1], P[2] + noise_point[:,2]])
			Q =np.concatenate((Q, np.array([[1,1,1]])), axis=0)
			T = point_to_point_minimization(P, Q)
			r = R.from_matrix(T[0:3,0:3])
			euler = r.as_euler('zyx', degrees=True)
			list_yaw.append(euler[0])
			list_pitch.append(euler[1])
			list_roll.append(euler[2])
			list_x.append(T[0,3])
			list_y.append(T[1,3])
			list_z.append(T[2,3])

		mean_x = np.mean(list_x)
		mean_y = np.mean(list_y)
		mean_z = np.mean(list_z)
		mean_roll = np.mean(list_roll)
		mean_pitch = np.mean(list_pitch)
		mean_yaw = np.mean(list_yaw)
		std_x = np.std(list_x)
		std_y = np.std(list_y)
		std_z = np.std(list_z)
		std_roll = np.std(list_roll)
		std_pitch = np.std(list_pitch)
		std_yaw = np.std(list_yaw)
		statistic_x.append(np.array([mean_x,std_x]))
		statistic_y.append(np.array([mean_y,std_y]))
		statistic_z.append(np.array([mean_z,std_z]))
		statistic_roll.append(np.array([mean_roll,std_roll]))
		statistic_pitch.append(np.array([mean_pitch,std_pitch]))
		statistic_yaw.append(np.array([mean_yaw,std_yaw]))
	return noise, statistic_x, statistic_y, statistic_z, statistic_roll, statistic_pitch, statistic_yaw

###################################################################################################
###################################################################################################

def find_nearest(array, value):
	array = np.asarray(array)
	idx = (np.abs(array - value)).argmin()
	return idx

# Function to compute the inter-gps distance
# Input:
# - gps_front: list of 1x4 array data from GPS front, [0]: timestamp (s), [1-2-3]: X, Y and Z (m)
# - gps_back: list of 1x4 array data from GPS back, [0]: timestamp (s), [1-2-3]: X, Y and Z (m)
# Output:
# - time_gps: list of timestamp for each tupple
# - distance_gps: list of inter-gps distance (m)
def distance_between_gps(gps_front, gps_back):
	time_gps = []
	distance_gps = []
	origin_time = 0
	index_arr = 0
	for i in range(0,len(gps_front)-1):
		index, time_index = findClosest(np.array(gps_back)[index_arr+1:-1,0], np.array(gps_front)[i,0])
		dist_time = abs(time_index-np.array(gps_front)[i,0])
		index_arr = index + index_arr
		if dist_time > 0.4:
			index = -1
		if index!=-1:
			gps_front_position = np.array([np.array(gps_front)[i,1],np.array(gps_front)[i,2],np.array(gps_front)[i,3]])
			gps_back_position = np.array([np.array(gps_back)[index_arr,1],np.array(gps_back)[index_arr,2],np.array(gps_back)[index_arr,3]])
			distance = np.linalg.norm(gps_front_position-gps_back_position)
			if(origin_time == 0):
				origin_time = gps_front[i][0]
			time_gps.append(gps_front[i][0]-origin_time)
			distance_gps.append(distance)
	return time_gps, distance_gps

def distance_between_gps_new(gps_1, gps_2):
	time_gps = []
	distance_gps = []
	gps_1_arr = np.array(gps_1)
	gps_2_arr = np.array(gps_2)
	for i in gps_1_arr:
		index = find_nearest(gps_2_arr[:,0], i[0])
		dist_time = abs(gps_2_arr[index,0]-i[0])
		if dist_time > 0.05:
			index = -1
		if index!=-1:
			gps_1_position = np.array([i[1],i[2],i[3]])
			gps_2_position = np.array([gps_2_arr[index,1],gps_2_arr[index,2],gps_2_arr[index,3]])
			distance = np.linalg.norm(gps_1_position-gps_2_position)
			time_gps.append(i[0])
			distance_gps.append(distance)
	return time_gps, distance_gps

# Function to compute the inter-gps distance
# Input:
# - gps_front: list of 1x4 array data from GPS, [0]: timestamp (s), [1-2-3]: X, Y and Z (m)
# - interpolated_time: list of timestamp for each of interpolation prism trajectories
# Output:
# - gps_front_sorted: list of 1x4 array data from GPS synchronize with prism data, [0]: timestamp (s), [1-2-3]: X, Y and Z (m)
def select_gps_data_synchronize_with_theodolites(gps_front, interpolated_time):
	gps_front_sorted = []
	for j in gps_front:
		for i in interpolated_time:
			if(j[0] >= i[0] and j[0] <= i[-1]):
				gps_front_sorted.append(j)
				break
	return gps_front_sorted

# Function to find not moving gps time
# Input:
# - not_moving_time: list of not moving time interval
# - gps_front: list of 1x4 array data from GPS front, [0]: timestamp (s), [1-2-3]: X, Y and Z (m)
# - gps_back: list of 1x4 array data from GPS back, [0]: timestamp (s), [1-2-3]: X, Y and Z (m)
# - number_points_limit: minimum number of GPS data for a not moving interval
# Output:
# - not_moving_gps_front, not_moving_gps_back: list of index for not moving gps position
def find_gps_not_moving_time(not_moving_time, gps_front, gps_back, number_points_limit):
	not_moving_gps_front = []
	not_moving_gps_back = []
	for i in not_moving_time:
		debut = i[0]
		fin = i[1]
		list_1 = []
		list_2 = []
		for j1 in range(0, len(gps_front)):
			if (gps_front[j1][0] >= debut and gps_front[j1][0] < fin):
				list_1.append(j1)
		for j2 in range(0, len(gps_back)):
			if (gps_back[j2][0] >= debut and gps_back[j2][0] < fin):
				list_2.append(j2)
		if (len(list_1) > number_points_limit and len(list_2) > number_points_limit):
			not_moving_gps_front.append(list_1)
			not_moving_gps_back.append(list_2)
	return not_moving_gps_front, not_moving_gps_back

# Function to compute the distance between not moving gps
# Input:
# - not_moving_gps_front, not_moving_gps_back: list of index for not moving gps position
# - gps_front: list of 1x4 array data from GPS front, [0]: timestamp (s), [1-2-3]: X, Y and Z (m)
# - gps_back: list of 1x4 array data from GPS back, [0]: timestamp (s), [1-2-3]: X, Y and Z (m)
# - time_limit: maximum time threshold for the time research tupple
# Output:
# - distance_gps: list of inter-gps distance (mm)
def distance_between_not_moving_gps(not_moving_gps_front, not_moving_gps_back, gps_back, gps_front, time_limit):
	distance_gps = []
	for i,j in zip(not_moving_gps_front,not_moving_gps_back):
		for k in i:
			index = research_index_for_time_gps(gps_back, j, gps_front[k][0], time_limit)
			if(index!=-1):
				gps_front_position = np.array([gps_front[k][1],gps_front[k][2],gps_front[k][3]])
				gps_back_position = np.array([gps_back[index][1],gps_back[index][2],gps_back[index][3]])
				distance = np.linalg.norm(gps_front_position-gps_back_position)*1000
				distance_gps.append(distance)
	return distance_gps


# Function to compute the inter-prism distance error during an experiment
# Input:
# file_name: csv file name containing prism poses in the total station frame.
# TF_list: list containing the 3 total stations TFs in order (1,2,3)
# Inter_prism_dist_list: list containing the different inter prism distances (12, 13, 23)
# Output:
# experiment error: list containing the inter prism distance error throughout the experiment
def inter_prism_distance_error_experiment(file_name, TF_list, Inter_prism_dist_list):
	if(len(Inter_prism_dist_list)!=0):
		trimble_1 = tu.read_prediction_data_experiment_csv_file(file_name + "1.csv")
		trimble_2 = tu.read_prediction_data_experiment_csv_file(file_name + "2.csv")
		trimble_3 = tu.read_prediction_data_experiment_csv_file(file_name + "3.csv")

		trimble_1 = TF_list[0]@trimble_1.T
		trimble_2 = TF_list[1]@trimble_2.T
		trimble_3 = TF_list[2]@trimble_3.T

		dist_12_t = Inter_prism_dist_list[0]
		dist_13_t = Inter_prism_dist_list[1]
		dist_23_t = Inter_prism_dist_list[2]

		error_inter_prism_dist = []
		for i, j, k in zip(trimble_1.T, trimble_2.T, trimble_3.T):
			error_inter_prism_dist.append(abs(np.linalg.norm(i[0:3] - j[0:3])-dist_12_t) * 1000)
			error_inter_prism_dist.append(abs(np.linalg.norm(i[0:3] - k[0:3])-dist_13_t) * 1000)
			error_inter_prism_dist.append(abs(np.linalg.norm(j[0:3] - k[0:3])-dist_23_t) * 1000)

		return error_inter_prism_dist
	else:
		return []
