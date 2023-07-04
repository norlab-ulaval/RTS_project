import matplotlib.pyplot as plt

# # import numpy as np
# # import random
# # import math
# # from numpy import linalg
# # from matplotlib import gridspec
# # from matplotlib.gridspec import GridSpec
# # import matplotlib as mpl

# # from mpl_toolkits.mplot3d import Axes3D
# # import seaborn as sns
# # from matplotlib.colors import BoundaryNorm
# # from matplotlib.ticker import MaxNLocator
# # from scripts.theodolite_utils import *
# # from scripts.theodolite_values import *
# # import matplotlib.patches as mpatches
# # from matplotlib.ticker import (MultipleLocator, FormatStrFormatter, AutoMinorLocator)
#
# ###################################################################################################
# ###################################################################################################
# # Plot raw data from theodolites
#
# # Function to plot the different trajectories found by each theodolite, and the theodolite position
# # Input:
# # - number_data: number of trajectories to plot
# # - trimble_1: list of points obtain by the theodolite 1
# # - trimble_2: list of points obtain by the theodolite 2
# # - trimble_3: list of points obtain by the theodolite 3
# # - plot_3d: param to set a 3D plot of the trajectories
# # - save_pdf: param if we want to save the figure in pdf
# # - file_pdf: name of the file to save the figure
# # - plot_equal: param to set the axis equal in 2D plot
# def plot_trajectories_prism_alone(number_data, trimble_1, trimble_2, trimble_3, plot_3d, save_pdf, file_pdf, plot_equal):
#     fig = plt.figure(figsize=(10,7))
#     if(plot_3d == 1):
#         ax = fig.add_subplot(111, projection='3d')
#         # Prisms trajectories in theodolite 1 frame
#         ax.scatter(trimble_1[0], trimble_1[1], trimble_1[2], c='r', marker='.', label="Prism with rasp 1")
#         if(number_data>1):
#             ax.scatter(trimble_2[0], trimble_2[1], trimble_2[2], c='b', marker='.', label="Prism with rasp 2")
#         if(number_data>2):
#             ax.scatter(trimble_3[0], trimble_3[1], trimble_3[2], c='g', marker='.', label="Prism with rasp 3")
#         ax.set_xlabel('X Label')
#         ax.set_ylabel('Y Label')
#         ax.set_zlabel('Z Label')
#         ax.legend(loc='best')
#         if(plot_equal==1):
#             ax.set_aspect('equal')
#     if(plot_3d == 0):
#         ax = fig.add_subplot(111)
#         # Prisms trajectories in theodolite 1 frame
#         ax.scatter(trimble_1[0], trimble_1[1], c='r', marker='.', label="Prism with rasp 1")
#         if(number_data>1):
#             ax.scatter(trimble_2[0], trimble_2[1], c='b', marker='.', label="Prism with rasp 2")
#         if(number_data>2):
#             ax.scatter(trimble_3[0], trimble_3[1], c='g', marker='.', label="Prism with rasp 3")
#         ax.set_xlabel('X Label')
#         ax.set_ylabel('Y Label')
#         ax.legend(loc='best')
#         if(plot_equal==1):
#             ax.set_aspect('equal')
#     plt.show()
#     #plt.gca().set_aspect('equal', adjustable='box')
#     fig.tight_layout()
#     if save_pdf == 1:
#         if plot_3d == 1:
#             fig.savefig(file_pdf, bbox_inches='tight')
#         else:
#             fig.savefig(file_pdf, bbox_inches='tight')

# Function to plot the different trajectories found by each theodolite, and the theodolite position
# Input:
# - number_data: number of trajectories to plot
# - trimble_1: list of points obtain by the theodolite 1
# - trimble_2: list of points obtain by the theodolite 2
# - trimble_3: list of points obtain by the theodolite 3
# - T_1_rasp: Frame of the theodolite 1, according to the chosen frame
# - T_12_rasp: Frame of the theodolite 2, according to the chosen frame
# - T_13_rasp: Frame of the theodolite 3, according to the chosen frame
# - plot_3d: param to set a 3D plot of the trajectories
# - save_pdf: param if we want to save the figure in pdf
# - file_pdf: name of the file to save the figure
# - plot_equal: param to set the axis equal in 2D plot
def plot_trajectories_prism(number_data, trimble_1, trimble_2, trimble_3, T_1_rasp, T_12_rasp, T_13_rasp, plot_3d, save_pdf, file_pdf, plot_equal):
    fig = plt.figure(figsize=(10,7))
    if(plot_3d == 1):
        ax = fig.add_subplot(111, projection='3d')
        # Calibration points in theodolite 1 frame
        #ax.scatter(x1, y1, z1, c='black', marker='o', s=200, label="Markers frame 1")
        # Theodolite positions in theodolite 1 frame
        ax.scatter(T_1_rasp[0,3], T_1_rasp[1,3], T_1_rasp[2,3], c='red', marker='*', s=200, label="Theodolite 1")
        if(number_data>1):
            ax.scatter(T_12_rasp[0,3], T_12_rasp[1,3], T_12_rasp[2,3], c='blue', marker='*', s=200, label="Theodolite 2")
        if(number_data>2):
            ax.scatter(T_13_rasp[0,3], T_13_rasp[1,3], T_13_rasp[2,3], c='green', marker='*', s=200, label="Theodolite 3")
        # Prisms trajectories in theodolite 1 frame
        ax.scatter(trimble_1[0], trimble_1[1], trimble_1[2], c='r', marker='.', label="Prism with rasp 1")
        if(number_data>1):
            ax.scatter(trimble_2[0], trimble_2[1], trimble_2[2], c='b', marker='.', label="Prism with rasp 2")
        if(number_data>2):
            ax.scatter(trimble_3[0], trimble_3[1], trimble_3[2], c='g', marker='.', label="Prism with rasp 3")
        ax.set_xlabel('x[m]')
        ax.set_ylabel('y[m]')
        ax.set_zlabel('Z Label')
        ax.legend(loc='best')
        if(plot_equal==1):
            ax.set_aspect('equal')
    if(plot_3d == 0):
        ax = fig.add_subplot(111)
        # Theodolite positions in theodolite 1 frame
        ax.scatter(T_1_rasp[0,3], T_1_rasp[1,3], c='red', marker='*', s=200, label="Theodolite 1")
        if(number_data>1):
            ax.scatter(T_12_rasp[0,3], T_12_rasp[1,3], c='blue', marker='*', s=200, label="Theodolite 2")
        if(number_data>2):
            ax.scatter(T_13_rasp[0,3], T_13_rasp[1,3], c='green', marker='*', s=200, label="Theodolite 3")
        # Prisms trajectories in theodolite 1 frame
        ax.scatter(trimble_1[0], trimble_1[1], c='r', marker='.', label="Prism with rasp 1")
        if(number_data>1):
            ax.scatter(trimble_2[0], trimble_2[1], c='b', marker='.', label="Prism with rasp 2")
        if(number_data>2):
            ax.scatter(trimble_3[0], trimble_3[1], c='g', marker='.', label="Prism with rasp 3")
        ax.set_xlabel('x[m]')
        ax.set_ylabel('y[m]')
        #ax.legend(loc='best')
        if(plot_equal==1):
            ax.set_aspect('equal')

        # # Theodolite positions in theodolite 1 frame
        # ax.scatter(T_1_rasp[1, 3], T_1_rasp[0, 3], c='red', marker='*', s=200, label="Theodolite 1")
        # if (number_data > 1):
        #     ax.scatter(T_12_rasp[1, 3], T_12_rasp[0, 3], c='blue', marker='*', s=200, label="Theodolite 2")
        # if (number_data > 2):
        #     ax.scatter(T_13_rasp[1, 3], T_13_rasp[0, 3], c='green', marker='*', s=200, label="Theodolite 3")
        # # Prisms trajectories in theodolite 1 frame
        # ax.scatter(trimble_1[1], trimble_1[0], c='r', marker='.', label="Prism with rasp 1")
        # if (number_data > 1):
        #     ax.scatter(trimble_2[1], trimble_2[0], c='b', marker='.', label="Prism with rasp 2")
        # if (number_data > 2):
        #     ax.scatter(trimble_3[1], trimble_3[0], c='g', marker='.', label="Prism with rasp 3")
        # ax.set_xlabel('y[m]')
        # ax.set_ylabel('x[m]')
        # #ax.legend(loc='best')
        # if (plot_equal == 1):
        #     ax.set_aspect('equal')
    plt.show()
    #plt.gca().set_aspect('equal', adjustable='box')
    fig.tight_layout()
    if save_pdf == 1:
        if plot_3d == 1:
            fig.savefig(file_pdf, bbox_inches='tight', transparent=True)
        else:
            fig.savefig(file_pdf, bbox_inches='tight', transparent=True)


def plot_trajectories_prism_GCP(number_data, trimble_1, trimble_2, trimble_3, T_1_rasp, T_12_rasp, T_13_rasp, GCP, plot_3d, save_pdf, file_pdf, plot_equal):
    fig = plt.figure(figsize=(10,7))
    if(plot_3d == 1):
        ax = fig.add_subplot(111, projection='3d')
        # Calibration points in theodolite 1 frame
        #ax.scatter(x1, y1, z1, c='black', marker='o', s=200, label="Markers frame 1")
        # Theodolite positions in theodolite 1 frame
        ax.scatter(T_1_rasp[0,3], T_1_rasp[1,3], T_1_rasp[2,3], c='red', marker='*', s=200, label="Theodolite 1")
        if(number_data>1):
            ax.scatter(T_12_rasp[0,3], T_12_rasp[1,3], T_12_rasp[2,3], c='blue', marker='*', s=200, label="Theodolite 2")
        if(number_data>2):
            ax.scatter(T_13_rasp[0,3], T_13_rasp[1,3], T_13_rasp[2,3], c='green', marker='*', s=200, label="Theodolite 3")
        # Prisms trajectories in theodolite 1 frame
        ax.scatter(trimble_1[0], trimble_1[1], trimble_1[2], c='r', marker='.', label="Prism with rasp 1")
        if(number_data>1):
            ax.scatter(trimble_2[0], trimble_2[1], trimble_2[2], c='b', marker='.', label="Prism with rasp 2")
        if(number_data>2):
            ax.scatter(trimble_3[0], trimble_3[1], trimble_3[2], c='g', marker='.', label="Prism with rasp 3")
        ax.set_xlabel('x[m]')
        ax.set_ylabel('y[m]')
        ax.set_zlabel('Z Label')
        ax.legend(loc='best')
        if(plot_equal==1):
            ax.set_aspect('equal')
    if(plot_3d == 0):
        ax = fig.add_subplot(111)
        # Theodolite positions in theodolite 1 frame
        # ax.scatter(T_1_rasp[0,3], T_1_rasp[1,3], c='red', marker='*', s=200, label="Theodolite 1")
        # if(number_data>1):
        #     ax.scatter(T_12_rasp[0,3], T_12_rasp[1,3], c='blue', marker='*', s=200, label="Theodolite 2")
        # if(number_data>2):
        #     ax.scatter(T_13_rasp[0,3], T_13_rasp[1,3], c='green', marker='*', s=200, label="Theodolite 3")
        # # Prisms trajectories in theodolite 1 frame
        # ax.scatter(trimble_1[0], trimble_1[1], c='r', marker='.', label="Prism with rasp 1")
        # if(number_data>1):
        #     ax.scatter(trimble_2[0], trimble_2[1], c='b', marker='.', label="Prism with rasp 2")
        # if(number_data>2):
        #     ax.scatter(trimble_3[0], trimble_3[1], c='g', marker='.', label="Prism with rasp 3")
        # ax.scatter(GCP[0, :], GCP[1, :], c='black', marker='.', s=200, label="GCP")
        # ax.set_xlabel('x[m]')
        # ax.set_ylabel('y[m]')
        # #ax.legend(loc='best')
        # if(plot_equal==1):
        #     ax.set_aspect('equal')

        # Theodolite positions in theodolite 1 frame
        # ax.scatter(T_1_rasp[1, 3], T_1_rasp[0, 3], c='red', marker='*', s=200, label="Robotic Total \nStation 1")
        # if (number_data > 1):
        #     ax.scatter(T_12_rasp[1, 3], T_12_rasp[0, 3], c='blue', marker='*', s=200, label="Robotic Total \nStation 2")
        # if (number_data > 2):
        #     ax.scatter(T_13_rasp[1, 3], T_13_rasp[0, 3], c='green', marker='*', s=200, label="Robotic Total \nStation 3")

        ax.scatter(T_1_rasp[1, 3], T_1_rasp[0, 3], c='red', marker='*', s=200, label="RTS 1")
        if (number_data > 1):
            ax.scatter(T_12_rasp[1, 3], T_12_rasp[0, 3], c='blue', marker='*', s=200, label="RTS 2")
        if (number_data > 2):
            ax.scatter(T_13_rasp[1, 3], T_13_rasp[0, 3], c='green', marker='*', s=200, label="RTS 3")
        # Prisms trajectories in theodolite 1 frame
        ax.scatter(trimble_1[1], trimble_1[0], c='r', marker='.', label="Prism 1")
        if (number_data > 1):
            ax.scatter(trimble_2[1], trimble_2[0], c='b', marker='.', label="Prism 2")
        if (number_data > 2):
            ax.scatter(trimble_3[1], trimble_3[0], c='g', marker='.', label="Prism 3")
        ax.scatter(GCP[1, :], GCP[0, :], c='black', marker='.', s=200, label="Ground control \npoints")
        ax.set_xlabel('y[m]')
        ax.set_ylabel('x[m]')

        # ax.scatter(T_1_rasp[0, 3], T_1_rasp[1, 3], c='red', marker='*', s=200, label="RTS 1")
        # if (number_data > 1):
        #     ax.scatter(T_12_rasp[0, 3], T_12_rasp[1, 3], c='blue', marker='*', s=200, label="RTS 2")
        # if (number_data > 2):
        #     ax.scatter(T_13_rasp[0, 3], T_13_rasp[1, 3], c='green', marker='*', s=200, label="RTS 3")
        # # Prisms trajectories in theodolite 1 frame
        # ax.scatter(trimble_1[0], trimble_1[1], c='r', marker='.', label="Prism 1")
        # if (number_data > 1):
        #     ax.scatter(trimble_2[0], trimble_2[1], c='b', marker='.', label="Prism 2")
        # if (number_data > 2):
        #     ax.scatter(trimble_3[0], trimble_3[1], c='g', marker='.', label="Prism 3")
        # ax.scatter(GCP[0, :], GCP[1, :], c='black', marker='.', s=200, label="Ground control \npoints")
        # ax.set_xlabel('x[m]')
        # ax.set_ylabel('y[m]')

        # ax.legend(loc='best')
        if (plot_equal == 1):
            ax.set_aspect('equal')
    plt.show()
    #plt.gca().set_aspect('equal', adjustable='box')
    fig.tight_layout()
    if save_pdf == 1:
        if plot_3d == 1:
            fig.savefig(file_pdf, bbox_inches='tight', transparent=True)
        else:
            fig.savefig(file_pdf, bbox_inches='tight', transparent=True)

# ###################################################################################################
# ###################################################################################################
# # Plot process data from theodolites
#
# # Function to plot one non-interpolated sub-trajectory result
# # Input:
# # - i: chosen number of the sub-trajectory in list_trajectories_split
# # - list_trajectories_split: list of the sub-trajectories
# # - trimble_1: list of points obtain by the theodolite 1
# # - trimble_2: list of points obtain by the theodolite 2
# # - trimble_3: list of points obtain by the theodolite 3
# def plot_subtrajectory_result(i, list_trajectories_split, trimble_1, trimble_2, trimble_3):
# 	debut_1 = list_trajectories_split[i][:,0]
# 	debut_2 = list_trajectories_split[i][:,1]
# 	debut_3 = list_trajectories_split[i][:,2]
# 	t1 = trimble_1[0:3,debut_1[0]:debut_1[1]]
# 	t2 = trimble_2[0:3,debut_2[0]:debut_2[1]]
# 	t3 = trimble_3[0:3,debut_3[0]:debut_3[1]]
# 	fig = plt.figure(figsize=(10,3))
# 	ax = fig.add_subplot(111)
# 	plt.plot(t1[0,:], t1[1,:], 'r', marker=".")
# 	plt.plot(t2[0,:], t2[1,:], 'b', marker=".")
# 	plt.plot(t3[0,:], t3[1,:], 'g', marker=".")
# 	ax.set_aspect('equal')
# 	plt.show()
#
# # Function to plot one interpolated sub-trajectory result
# # Input:
# # - i: chosen number of the sub-trajectory in interpolated_trajectories
# # - interpolated_trajectories: list of the interpolated sub-trajectories
# def plot_interpolated_subtrajectory_result(k, interpolated_trajectories):
# 	plt.figure()
# 	plt.plot(interpolated_trajectories[k][0][0], interpolated_trajectories[k][0][1], 'r', marker=".")
# 	plt.plot(interpolated_trajectories[k][1][0], interpolated_trajectories[k][1][1], 'g', marker=".")
# 	plt.plot(interpolated_trajectories[k][2][0], interpolated_trajectories[k][2][1], 'b', marker=".")
# 	plt.show()
#
# # Function to plot one ptp minimzation from one interpolated sub-trajectory
# # Input:
# # - Pose_lidar: list the lidar poses
# # - Prism_corrected: list of each prism corrected poses
# # - interpolated_trajectories: list of the interpolated sub-trajectories
# # - i: chosen number of the interpolated trajectories
# # - j: pose of the lidar, first pose of the corrected prism
# # - k: coreected prism pose just after j
# def ptp_plot(Pose_lidar, Prism_corrected, interpolated_trajectories, i, j, k):
# 	Prism_corrected_arr =np.array(Prism_corrected)
# 	Pose_lidar_arr = np.array(Pose_lidar)
# 	fig = plt.figure(figsize=(9,6))
# 	ax = fig.add_subplot(311)
# 	ax.scatter(interpolated_trajectories[i][0][0,j], interpolated_trajectories[i][0][1,j], c='orange', marker='o')
# 	ax.scatter(interpolated_trajectories[i][1][0,j], interpolated_trajectories[i][1][1,j], c='orange', marker='o')
# 	ax.scatter(interpolated_trajectories[i][2][0,j], interpolated_trajectories[i][2][1,j], c='orange', marker='o')
# 	ax.scatter(interpolated_trajectories[i][0][0,k], interpolated_trajectories[i][0][1,k], c='cyan', marker='o')
# 	ax.scatter(interpolated_trajectories[i][1][0,k], interpolated_trajectories[i][1][1,k], c='cyan', marker='o')
# 	ax.scatter(interpolated_trajectories[i][2][0,k], interpolated_trajectories[i][2][1,k], c='cyan', marker='o')
# 	ax.scatter(Prism_corrected_arr[j,0], Prism_corrected_arr[j,1], c='black', marker='.')
# 	ax.scatter(Pose_lidar_arr[j,0,3], Pose_lidar_arr[j,1,3], c='blue', marker='*', label="Reference")
# 	ax.scatter(Prism_corrected_arr[k,0], Prism_corrected_arr[k,1], c='green', marker='.')
# 	ax.scatter(Pose_lidar_arr[k,0,3], Pose_lidar_arr[k,1,3], c='r', marker='*', label="Reference")
# 	ax.set_aspect('equal')
# 	ax.set_xlabel('X Label')
# 	ax.set_ylabel('Y Label')
# 	ax = fig.add_subplot(312)
# 	ax.scatter(interpolated_trajectories[i][0][0,j], interpolated_trajectories[i][0][2,j], c='orange', marker='o')
# 	ax.scatter(interpolated_trajectories[i][1][0,j], interpolated_trajectories[i][1][2,j], c='orange', marker='o')
# 	ax.scatter(interpolated_trajectories[i][2][0,j], interpolated_trajectories[i][2][2,j], c='orange', marker='o')
# 	ax.scatter(interpolated_trajectories[i][0][0,k], interpolated_trajectories[i][0][2,k], c='cyan', marker='o')
# 	ax.scatter(interpolated_trajectories[i][1][0,k], interpolated_trajectories[i][1][2,k], c='cyan', marker='o')
# 	ax.scatter(interpolated_trajectories[i][2][0,k], interpolated_trajectories[i][2][2,k], c='cyan', marker='o')
# 	ax.scatter(Prism_corrected_arr[j,0], Prism_corrected_arr[j,2], c='black', marker='.')
# 	ax.scatter(Pose_lidar_arr[j,0,3], Pose_lidar_arr[j,2,3], c='blue', marker='*', label="Reference")
# 	ax.scatter(Prism_corrected_arr[k,0], Prism_corrected_arr[k,2], c='green', marker='.')
# 	ax.scatter(Pose_lidar_arr[k,0,3], Pose_lidar_arr[k,2,3], c='r', marker='*', label="Reference")
# 	ax.set_aspect('equal')
# 	ax.set_xlabel('X Label')
# 	ax.set_ylabel('Z Label')
# 	ax = fig.add_subplot(313)
# 	ax.scatter(interpolated_trajectories[i][0][1,j], interpolated_trajectories[i][0][2,j], c='orange', marker='o')
# 	ax.scatter(interpolated_trajectories[i][1][1,j], interpolated_trajectories[i][1][2,j], c='orange', marker='o')
# 	ax.scatter(interpolated_trajectories[i][2][1,j], interpolated_trajectories[i][2][2,j], c='orange', marker='o')
# 	ax.scatter(interpolated_trajectories[i][0][1,k], interpolated_trajectories[i][0][2,k], c='cyan', marker='o')
# 	ax.scatter(interpolated_trajectories[i][1][1,k], interpolated_trajectories[i][1][2,k], c='cyan', marker='o')
# 	ax.scatter(interpolated_trajectories[i][2][1,k], interpolated_trajectories[i][2][2,k], c='cyan', marker='o')
# 	ax.scatter(Prism_corrected_arr[j,1], Prism_corrected_arr[j,2], c='black', marker='.')
# 	ax.scatter(Pose_lidar_arr[j,1,3], Pose_lidar_arr[j,2,3], c='blue', marker='*', label="Reference")
# 	ax.scatter(Prism_corrected_arr[k,1], Prism_corrected_arr[k,2], c='green', marker='.')
# 	ax.scatter(Pose_lidar_arr[k,1,3], Pose_lidar_arr[k,2,3], c='r', marker='*', label="Reference")
# 	ax.set_aspect('equal')
# 	ax.set_xlabel('Y Label')
# 	ax.set_ylabel('Z Label')
# 	plt.show()
# 	fig.tight_layout()
#
# # Function to plot the average inter-prism error according to the list1 and list2. Scatter plot of the result
# # Input:
# # - mean_error_prisms_list: list of the average inter-prism error
# # - list_1: list for X axis
# # - list_2: list for the Y axis
# def scatter_plot_result(mean_error_prisms_list, list_1, list_2):
# 	min_error  = min(mean_error_prisms_list)
# 	max_error = max(mean_error_prisms_list)
# 	arr1 = np.array(list_1)
# 	arr2 = np.array(list_2)
# 	mean_error_prisms_arr = np.array(mean_error_prisms_list)
# 	fig = plt.figure(figsize=(8, 6))
# 	cm = plt.cm.get_cmap('jet')
# 	sc = plt.scatter(arr1, arr2, vmin=min_error, vmax=max_error, c=mean_error_prisms_arr, cmap=cm, s=10)
# 	plt.colorbar(sc)
# 	plt.xscale('symlog')
# 	plt.show()
#
# ###################################################################################################
# ###################################################################################################
# # Plot theodolite data with lidar result
#
# # Function to plot the different trajectories found by each theodolite, the theodolite position and the lidar position
# # Input:
# # - number_data: number of trajectories to plot
# # - trimble_1: list of points obtain by the theodolite 1
# # - trimble_2: list of points obtain by the theodolite 1
# # - trimble_3: list of points obtain by the theodolite 1
# # - T_1_rasp: Frame of the theodolite 1, according to the chosen frame
# # - T_12_rasp: Frame of the theodolite 2, according to the chosen frame
# # - T_13_rasp: Frame of the theodolite 3, according to the chosen frame
# # - T_lidar: Frame of the lidar position, according to the chosen frame
# # - plot_3d: param to set a 3D plot of the trajectories
# # - save_pdf: param if we want to save the figure in pdf
# # - file_pdf: name of the file to save the figure
# # - plot_equal: param to set the axis equal in 2D plot
# def plot_trajectories_prism_with_lidar_and_theodolites(number_data, trimble_1, trimble_2, trimble_3, T_1_rasp, T_12_rasp, T_13_rasp, T_lidar, plot_3d, save_pdf, file_pdf, plot_equal):
# 	T_lidar_arr = np.array(T_lidar)
# 	fig = plt.figure(figsize=(10,7))
# 	if(plot_3d == 1):
# 		ax = fig.add_subplot(111, projection='3d')
# 		ax.scatter(trimble_1[0], trimble_1[1], trimble_1[2], c='r', marker='.', label="Prism with rasp 1")
# 		if(number_data>1):
# 			ax.scatter(trimble_2[0], trimble_2[1], trimble_2[2], c='b', marker='.', label="Prism with rasp 2")
# 		if(number_data>2):
# 			ax.scatter(trimble_3[0], trimble_3[1], trimble_3[2], c='g', marker='.', label="Prism with rasp 3")
# 		ax.scatter(T_lidar_arr[:,0,3], T_lidar_arr[:,1,3], T_lidar_arr[:,2,3], c='black', marker='.')
# 		ax.set_xlabel('X Label')
# 		ax.set_ylabel('Y Label')
# 		ax.set_zlabel('Z Label')
# 		ax.legend(loc='best')
# 		if(plot_equal==1):
# 			ax.set_aspect('equal')
# 	if(plot_3d == 0):
# 		ax = fig.add_subplot(111)
# 		ax.scatter(T_1_rasp[0,3], T_1_rasp[1,3], c='red', marker='*', s=100, label="Theodolite 1")
# 		ax.scatter(trimble_1[0], trimble_1[1], c='r', marker='.', label="Prism 1")
# 		if(number_data>1):
# 			ax.scatter(T_12_rasp[0,3], T_12_rasp[1,3], c='blue', marker='*', s=100, label="Theodolite 2")
# 			ax.scatter(trimble_2[0], trimble_2[1], c='b', marker='.', label="Prism 2")
# 		if(number_data>2):
# 			ax.scatter(T_13_rasp[0,3], T_13_rasp[1,3], c='green', marker='*', s=100, label="Theodolite 3")
# 			ax.scatter(trimble_3[0], trimble_3[1], c='g', marker='.', label="Prism 3")
# 		ax.scatter(T_lidar_arr[:,0,3], T_lidar_arr[:,1,3], c='black', marker='.', label="Lidar")
# 		ax.set_xlabel('X(m)')
# 		ax.set_ylabel('Y(m)')
# 		plt.xlim([-7.5, 7.5])
# 		plt.ylim([-2, 13])
# 		ax.legend(loc='best')
# 		if(plot_equal==1):
# 			ax.set_aspect('equal')
# 	plt.show()
# 	#plt.gca().set_aspect('equal', adjustable='box')
# 	fig.tight_layout()
# 	if save_pdf == 1:
# 		if plot_3d == 1:
# 			fig.savefig(file_pdf, bbox_inches='tight')
# 		else:
# 			fig.savefig(file_pdf, bbox_inches='tight')
#
# # Function to plot the different trajectories found by each theodolite, and the lidar position
# # Input:
# # - number_data: number of trajectories to plot
# # - trimble_1: list of points obtain by the theodolite 1
# # - trimble_2: list of points obtain by the theodolite 2
# # - trimble_3: list of points obtain by the theodolite 3
# # - T_1_rasp: Frame of the theodolite 1, according to the chosen frame
# # - T_12_rasp: Frame of the theodolite 2, according to the chosen frame
# # - T_13_rasp: Frame of the theodolite 3, according to the chosen frame
# # - T_lidar: Frame of the lidar
# # - plot_3d: param to set a 3D plot of the trajectories
# # - save_pdf: param if we want to save the figure in pdf
# # - file_pdf: name of the file to save the figure
# # - plot_equal: param to set the axis equal in 2D plot
# def plot_trajectories_prism_with_lidar(number_data, trimble_1, trimble_2, trimble_3, T_1_rasp, T_12_rasp, T_13_rasp, T_lidar, plot_3d, save_pdf, file_pdf, plot_equal):
# 	T_lidar_arr = np.array(T_lidar)
# 	fig = plt.figure(figsize=(10,7))
# 	if(plot_3d == 1):
# 		ax = fig.add_subplot(111, projection='3d')
# 		ax.scatter(trimble_1[0], trimble_1[1], trimble_1[2], c='r', marker='.', label="Prism with rasp 1")
# 		if(number_data>1):
# 			ax.scatter(trimble_2[0], trimble_2[1], trimble_2[2], c='b', marker='.', label="Prism with rasp 2")
# 		if(number_data>2):
# 			ax.scatter(trimble_3[0], trimble_3[1], trimble_3[2], c='g', marker='.', label="Prism with rasp 3")
# 		ax.scatter(T_lidar_arr[:,0,3], T_lidar_arr[:,1,3], T_lidar_arr[:,2,3], c='black', marker='.')
# 		ax.set_xlabel('X Label')
# 		ax.set_ylabel('Y Label')
# 		ax.set_zlabel('Z Label')
# 		ax.legend(loc='best')
# 		if(plot_equal==1):
# 			ax.set_aspect('equal')
# 	if(plot_3d == 0):
# 		ax = fig.add_subplot(111)
# 		ax.scatter(trimble_1[0], trimble_1[1], c='r', marker='.', label="Prism with rasp 1")
# 		if(number_data>1):
# 			ax.scatter(trimble_2[0], trimble_2[1], c='b', marker='.', label="Prism with rasp 2")
# 		if(number_data>2):
# 			ax.scatter(trimble_3[0], trimble_3[1], c='g', marker='.', label="Prism with rasp 3")
# 		ax.scatter(T_lidar_arr[:,0,3], T_lidar_arr[:,1,3], c='black', marker='.')
# 		ax.set_xlabel('X Label')
# 		ax.set_ylabel('Y Label')
# 		ax.legend(loc='best')
# 		if(plot_equal==1):
# 			ax.set_aspect('equal')
# 	plt.show()
# 	#plt.gca().set_aspect('equal', adjustable='box')
# 	fig.tight_layout()
#
# 	if save_pdf == 1:
# 		if plot_3d == 1:
# 			fig.savefig(file_pdf, bbox_inches='tight')
# 		else:
# 			fig.savefig(file_pdf, bbox_inches='tight')
#
# ###################################################################################################
# ###################################################################################################
# # Plot histograms
#
# # Function to plot the histogram of one inter-prism error
# # Input:
# # - distance_error: list of the inter-prism error
# # - save_fig: param if we want to save the figure in pdf
# # - name_file: name of the file to save the figure
# # - marker: option to set the axis label according to what should be plot, 0: inter-prism error, 1: inter-prism distance, 2: inter-gps distance
# def plot_histogram(distance_error, save_fig, name_file, marker):
# 	scale_x_min = min(distance_error)-1
# 	scale_x_max = max(distance_error)+1
# 	x = np.linspace(scale_x_min,scale_x_max,200) #for plot distribution
# 	mean = np.mean(distance_error)
# 	std = np.std(distance_error)
# 	gaussian_density = normal_dist(x, mean, std)
# 	fig, ax = plt.subplots(1,1,figsize=(6,5))
# 	ax = sns.distplot(distance_error, kde=False, color='grey', bins=20, hist_kws={'edgecolor':'black', 'alpha':0.5})
# 	# Creating another Y axis
# 	second_ax = ax.twinx()
# 	#Plotting kde without hist on the second Y axis
# 	plt.plot(x, gaussian_density, color = 'red')
# 	#sns.distplot(distance_error, ax=second_ax, kde=True, hist=False, color='r')
# 	#Removing Y ticks from the second axis
# 	second_ax.set_yticks([])
# 	if(marker == 0):
# 		axis_label = 'Error distance (mm),\n $\mu$ = ' + str(round(mean, 2)) + 'mm, $\sigma$ = ' + str(round(std, 2)) + 'mm'
# 	if(marker == 1):
# 		axis_label = 'Distance between prism (mm),\n $\mu$ = ' + str(round(mean, 2)) + 'mm, $\sigma$ = ' + str(round(std, 2)) + 'mm'
# 	if(marker == 2):
# 		axis_label = 'Distance between gps (mm),\n $\mu$ = ' + str(round(mean, 2)) + 'mm, $\sigma$ = ' + str(round(std, 2)) + 'mm'
# 	ax.set_xlabel(axis_label)
# 	ax.set_ylabel('Counts')
# 	fig.tight_layout()
# 	plt.show()
# 	if(save_fig == True):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# # Function to plot the histograms of each inter-prism error
# # Input:
# # - distance_12, distance_13, distance_23: list of the inter-prism error for each distance
# # - save_fig: param if we want to save the figure in pdf
# # - name_file: name of the file to save the figure
# def subplot_histogram(distance_12, distance_13, distance_23, save_fig, name_file):
# 	fig = plt.figure(figsize=(7, 3))
# 	#fig, ax = plt.subplots(1, 3, figsize=(8, 10))
# 	ax = fig.add_subplot(1,3,1)
# 	scale_x_min_12 = np.min(distance_12)-1
# 	scale_x_max_12 = np.max(distance_12)+1
# 	scale_x_min_13 = np.min(distance_13)-4
# 	scale_x_max_13 = np.max(distance_13)+4
# 	scale_x_min_23 = np.min(distance_23)-1
# 	scale_x_max_23 = np.max(distance_23)+1
# 	x_12 = np.linspace(scale_x_min_12,scale_x_max_12,200)
# 	mean_12 = np.mean(distance_12)
# 	std_12 = np.std(distance_12)
# 	gaussian_density_12 = normal_dist(x_12, mean_12, std_12)
# 	x_13 = np.linspace(scale_x_min_13,scale_x_max_13,200)
# 	mean_13 = np.mean(distance_13)
# 	std_13 = np.std(distance_13)
# 	gaussian_density_13 = normal_dist(x_13, mean_13, std_13)
# 	x_23 = np.linspace(scale_x_min_23,scale_x_max_23,200)
# 	mean_23 = np.mean(distance_23)
# 	std_23 = np.std(distance_23)
# 	gaussian_density_23 = normal_dist(x_23, mean_23, std_23)
# 	scale_y_max = 25
# 	ax.set_ylim([0,scale_y_max])
# 	ax = sns.distplot(distance_12, kde=False, color='dimgray', bins=20, hist_kws={'edgecolor':'None', 'alpha':0.7})
# 	# Creating another Y axis
# 	second_ax = ax.twinx()
# 	plt.vlines(987, ymin = 0, ymax = scale_y_max, color = "goldenrod")
# 	second_ax.set_ylim([0,0.14])
# 	#Plotting kde without hist on the second Y axis
# 	#plt.plot(x_12, gaussian_density_12, color = 'red')
# 	#Removing Y ticks from the second axis
# 	second_ax.set_yticks([])
# 	axis_label = '\n $\mu_{12}$ = ' + str(round(mean_12, 2)) + 'mm \n $\sigma_{12}$ = ' + str(round(std_12, 2)) + 'mm'
# 	ax.set_xlabel(axis_label)
# 	ax.set_ylabel('Counts')
# 	ax.text(987.1, 20, r'987mm', fontsize=12, color='goldenrod')
#
# 	ax = fig.add_subplot(1,3,2)
# 	ax.set_ylim([0,scale_y_max])
# 	ax = sns.distplot(distance_13, kde=False, color='dimgray', bins=20, hist_kws={'edgecolor':'None', 'alpha':0.7})
# 	# Creating another Y axis
# 	second_ax = ax.twinx()
# 	plt.vlines(681, ymin = 0, ymax = scale_y_max, color = "goldenrod")
# 	second_ax.set_ylim([0,0.017])
# 	#Plotting kde without hist on the second Y axis
# 	#plt.plot(x_13, gaussian_density_13, color = 'red')
# 	#Removing Y ticks from the second axis
# 	second_ax.set_yticks([])
# 	axis_label = 'Distance between prisms [mm] \n' + '$\mu_{13}$ = ' + str(round(mean_13, 2)) + 'mm \n $\sigma_{13}$ = ' + str(round(std_13, 2)) + 'mm'
# 	ax.set_xlabel(axis_label)
# 	#ax.set_ylabel('Counts')
# 	align_yaxis(ax, 0, second_ax, 0)
# 	ax.text(681.1, 20, r'681mm', fontsize=12, color='goldenrod')
#
# 	ax = fig.add_subplot(1,3,3)
# 	ax.set_ylim([0,scale_y_max])
# 	ax = sns.distplot(distance_23, kde=False, color='dimgray', bins=20, hist_kws={'edgecolor':'None', 'alpha':0.7})
# 	# Creating another Y axis
# 	second_ax = ax.twinx()
# 	plt.vlines(815, ymin = 0, ymax = scale_y_max, color = "goldenrod")
# 	second_ax.set_ylim([0,0.26])
# 	#Plotting kde without hist on the second Y axis
# 	#plt.plot(x_23, gaussian_density_23, color = 'red')
# 	#Removing Y ticks from the second axis
# 	second_ax.set_yticks([])
# 	axis_label = '\n $\mu_{23}$ = ' + str(round(mean_23, 2)) + 'mm \n $\sigma_{23}$ = ' + str(round(std_23, 2)) + 'mm'
# 	ax.set_xlabel(axis_label)
# 	ax.text(815.1, 20, r'815mm', fontsize=12, color='goldenrod')
# 	#ax.set_ylabel('Counts')
# 	plt.rc('font', size=12)
# 	fig.subplots_adjust(bottom=0.4, wspace=0.3)
#
# 	#fig.tight_layout()
# 	plt.show()
# 	if(save_fig == True):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# ###################################################################################################
# ###################################################################################################
# # Plot speed/acceleration and theodolite data
#
# # Function to plot the inter-prism error with the linear/rotation speed of the robot
# # Input:
# # - speed_axis: list of speed interval for the plot
# # - mean_list: list of the 3 average inter-distance value according to the speed
# # - std_list: list of the 3 standard deviation inter-distance value according to the speed
# # - true_value_list: list of the true inter-prism distance for comparison
# # - save_fig: param if we want to save the figure in pdf
# # - name_file: name of the file to save the figure
# # - angular: option to set the X axis label, 0: linear speed, 1: angular speed
# def plot_statistic_dynamic_interpolation(speed_axis, mean_list, std_list, true_value_list, save_fig, name_file, angular):
# 	speed_axis_arr = np.array(speed_axis)
# 	mean_12_arr = np.array(mean_list[0])
# 	mean_13_arr = np.array(mean_list[1])
# 	mean_23_arr = np.array(mean_list[2])
# 	std_12_arr = np.array(std_list[0])
# 	std_13_arr = np.array(std_list[1])
# 	std_23_arr = np.array(std_list[2])
# 	true_value_12_arr = np.array(true_value_list[0])
# 	true_value_13_arr = np.array(true_value_list[1])
# 	true_value_23_arr = np.array(true_value_list[2])
#
# 	fig, (ax0, ax1, ax2) = plt.subplots(nrows=3, sharex=True, figsize=(12,8))
#
# 	ax0.plot(speed_axis_arr, mean_12_arr, color = 'red', linewidth=2.0, label="Bias")
# 	ax0.plot(speed_axis_arr, true_value_12_arr, color = 'black', linewidth=2.0, label="True value")
# 	ax0.plot(speed_axis_arr, mean_12_arr-std_12_arr, color = 'red', alpha=0.3)
# 	ax0.plot(speed_axis_arr, mean_12_arr+std_12_arr, color = 'red', alpha=0.3)
# 	ax0.fill_between(speed_axis_arr,mean_12_arr-std_12_arr, mean_12_arr+std_12_arr, color = 'red', alpha=0.3)
# 	ax0.legend()
# 	#ax0.set_xlabel('Speed of warthog (m/s)')
# 	ax0.set_ylabel('Distance between \n prism 1 to 2 (mm)')
#
# 	ax1.plot(speed_axis_arr, mean_13_arr, color = 'red', linewidth=2.0, label="Bias")
# 	ax1.plot(speed_axis_arr, true_value_13_arr, color = 'black', linewidth=2.0, label="True value")
# 	ax1.plot(speed_axis_arr, mean_13_arr-std_13_arr, color = 'red', alpha=0.3)
# 	ax1.plot(speed_axis_arr, mean_13_arr+std_13_arr, color = 'red', alpha=0.3)
# 	ax1.fill_between(speed_axis_arr,mean_13_arr-std_13_arr, mean_13_arr+std_13_arr, color = 'red', alpha=0.3)
# 	ax1.legend()
# 	#ax1.set_xlabel('Speed of warthog (m/s)')
# 	ax1.set_ylabel('Distance between \n prism 1 to 3 (mm)')
#
# 	ax2.plot(speed_axis_arr, mean_23_arr, color = 'red', linewidth=2.0, label="Bias")
# 	ax2.plot(speed_axis_arr, true_value_23_arr, color = 'black', linewidth=2.0, label="True value")
# 	ax2.plot(speed_axis_arr, mean_23_arr-std_23_arr, color = 'red', alpha=0.3)
# 	ax2.plot(speed_axis_arr, mean_23_arr+std_23_arr, color = 'red', alpha=0.3)
# 	ax2.fill_between(speed_axis_arr,mean_23_arr-std_23_arr, mean_23_arr+std_23_arr, color = 'red', alpha=0.3)
# 	ax2.legend()
# 	if(angular != True):
# 		ax2.set_xlabel('Speed of warthog (m/s)')
# 	else:
# 		ax2.set_xlabel('Angular velocity around Z (rad/s)')
# 	ax2.set_ylabel('Distance between \n prism 2 to 3 (mm)')
# 	fig.tight_layout()
# 	plt.show()
# 	if(save_fig == True):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# # Function to align the x axis to the same origin
# # Input:
# # - ax1: ax class number 1
# # - v1: origin of the ax number 1
# # - ax2: ax class number 2
# # - v2: origin of the ax number 2
# def align_yaxis(ax1, v1, ax2, v2):
#     """adjust ax2 ylimit so that v2 in ax2 is aligned to v1 in ax1"""
#     _, y1 = ax1.transData.transform((0, v1))
#     _, y2 = ax2.transData.transform((0, v2))
#     inv = ax2.transData.inverted()
#     _, dy = inv.transform((0, 0)) - inv.transform((0, y1-y2))
#     miny, maxy = ax2.get_ylim()
#     ax2.set_ylim(miny+dy, maxy+dy)
#
# # Function to plot the inter-prism error and the linear/rotation speed in two subplot
# # Input:
# # - interpolated_trajectories: list of the interpolated trajectories
# # - interpolated_time: list of the time for each interpolated trajectories
# # - Dist_prism_12, Dist_prism_13, Dist_prism_23: list of inter-prism distance
# # - speed: list of the speed of the robot through time
# # - angular_speed: list of the angular speed around Z axis of the robot through time
# # - save_fig: param if we want to save the figure in pdf
# # - name_file: name of the file to save the figure
# def plot_speed_and_error(interpolated_trajectories, interpolated_time, Dist_prism_12, Dist_prism_13, Dist_prism_23, speed, angular_speed, save_fig, name_file):
# 	time_value = []
# 	dist_12_value = []
# 	dist_13_value = []
# 	dist_23_value = []
# 	compteur = 0
# 	origin_time = 0
# 	for i,j in zip(interpolated_trajectories, interpolated_time):
# 		for k in range(0,len(j)):
# 			if(compteur == 0):
# 				origin_time = j[k]
# 				compteur=compteur+1
# 			time_value.append(j[k]-origin_time)
# 			position_1 = i[0][0:3,k]
# 			position_2 = i[1][0:3,k]
# 			position_3 = i[2][0:3,k]
# 			dist_12_value.append(abs(np.linalg.norm(position_1-position_2)*1000-Dist_prism_12))
# 			dist_13_value.append(abs(np.linalg.norm(position_1-position_3)*1000-Dist_prism_13))
# 			dist_23_value.append(abs(np.linalg.norm(position_2-position_3)*1000-Dist_prism_23))
#
# 	time_value_arr = np.array(time_value)
# 	dist_12_value_arr = np.array(dist_12_value)
# 	dist_13_value_arr = np.array(dist_13_value)
# 	dist_23_value_arr = np.array(dist_23_value)
# 	begin_time = 80
# 	#end_time = time_value_arr[-1]
# 	end_time = 140
# 	print(end_time)
#
# 	fig = plt.figure(figsize=(6,5))
# 	ax = fig.add_subplot(2,1,1)
#
# 	ax.plot(time_value_arr, dist_12_value_arr, 'goldenrod', label="$\epsilon_{12}$")
# 	ax.legend(loc='upper left')
# 	ax.plot(time_value_arr, dist_13_value_arr, 'darkblue', label="$\epsilon_{13}$")
# 	ax.legend(loc='upper center')
# 	ax.plot(time_value_arr, dist_23_value_arr, 'sienna', label="$\epsilon_{23}$")
# 	ax.set_xlim([begin_time, end_time])
# 	ax.legend(loc='upper right')
# 	ax.set_ylabel("Inter-prism \nerrors [mm]")
# 	#plt.yscale('symlog')
# 	ax.set_ylim([0, 240])
#
# 	ax1 = fig.add_subplot(2,1,2, sharex=ax)
# 	ax1.plot(np.array(speed)[:,0]-origin_time, abs(np.array(speed)[:,1]), 'sienna', label="Linear speed")
# 	ax1.set_xlabel("Time [s]")
# 	ax1.set_ylabel("Linear speed [m/s]")
# 	ax1.set_ylim([0, 2])
#
# 	ax2=ax1.twinx()
# 	ax2.plot(np.array(angular_speed)[:,0]-origin_time, abs(np.array(angular_speed)[:,1]), 'darkblue', label="Angular speed")
# 	ax2.set_ylabel("Angular speed around \n Z axis [rad/s]")
# 	ax2.set_xlim([begin_time, end_time])
# 	ax2.legend(loc='upper right')
# 	ax2.set_ylim([0, 0.6])
# 	align_yaxis(ax1, 0, ax2, 0)
# 	ax1.legend(loc='upper left')
# 	plt.rc('font', size=12)
#
# 	fig.tight_layout()
# 	plt.show()
# 	if(save_fig):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# # Function to plot the inter-prism error according to linear speed/accel and angular speed around Z axis
# # Input:
# # - dx: resolution of the x axis for the grid
# # - dy: resolution of the y axis for the grid
# # - mean_error_prisms_list: mean of the inter-prism error
# # - list_1: list of the linear speed/acceleration of the robot
# # - list_2: list of the angular speed around Z axis of the robot
# # - save_fig: param if we want to save the figure in pdf
# # - name_file: name of the file to save the figure
# # - Speed: option to set the x axis label, True: linear speed, False: acceleration
# # - method: option to set the calculation of the value in each grid, 0: mean of average inter-prism error, 1: max of average inter-prism error
# def plot_grid_speed_angular_result(dx, dy, mean_error_prisms_list, list_1, list_2, save_fig, name_file, Speed, method):
# 	min_error  = min(mean_error_prisms_list)
# 	max_error = max(mean_error_prisms_list)
# 	min_speed = min(list_1)
# 	max_speed = max(list_1)
# 	min_angular = min(list_2)
# 	max_angular = max(list_2)
# 	arr1 = np.array(list_1)
# 	arr2 = np.array(list_2)
# 	mean_error_prisms_arr = np.array(mean_error_prisms_list)
#
# 	# generate 2 2d grids for the x & y bounds
# 	x, y = np.mgrid[slice(min_speed, max_speed + dx, dx),
# 		              slice(min_angular, max_angular + dy, dy)]
#
# 	sort_value_arr = np.zeros((2,x.shape[0],x.shape[1]))
# 	for i in range(0, len(arr1)):
# 		index_x = -1
# 		index_y = -1
# 		for j in range(0,len(x)-1):
# 			if(arr1[i]<x[j+1][0] and arr1[i]>=x[j][0]):
# 				index_x = j
# 				break
# 		for k in range(0,len(y[0])-1):
# 			if(arr2[i]<y[0][k+1] and arr2[i]>=y[0][k]):
# 				index_y = k
# 				break
# 		if(index_x != -1 and index_y != -1):
# 			if(method == 0):
# 				sort_value_arr[0, index_x, index_y] = sort_value_arr[0, index_x, index_y] + mean_error_prisms_arr[i]
# 				sort_value_arr[1, index_x, index_y] = sort_value_arr[1, index_x, index_y] + 1
# 			else:
# 				sort_value_arr[0, index_x, index_y] = max(sort_value_arr[0, index_x, index_y], mean_error_prisms_arr[i])
# 				sort_value_arr[1, index_x, index_y] = sort_value_arr[1, index_x, index_y]+1
#
# 		z = np.zeros((x.shape[0],x.shape[1]))
# 		for i in range(0,z.shape[0]):
# 			for j in range(0,z.shape[1]):
# 				if(sort_value_arr[1, i,j]!=0):
# 					if(method == 0):
# 						z[i,j]=sort_value_arr[0, i,j]/sort_value_arr[1, i,j]
# 					else:
# 						z[i,j]=sort_value_arr[0, i,j]
# 				else:
# 						z[i,j]= 'nan'
#
# 	# x and y are bounds, so z should be the value *inside* those bounds.
# 	# Therefore, remove the last value from the z array.
# 	z = z[:-1, :-1]
# 	levels = MaxNLocator(nbins=10).tick_values(z.min(), z.max())
# 	# pick the desired colormap, sensible levels, and define a normalization
# 	# instance which takes data values and translates those into levels.
# 	cmap = plt.get_cmap('inferno')
# 	norm = BoundaryNorm(levels, ncolors=cmap.N, clip=True)
#
# 	fig, ax = plt.subplots(nrows=1, figsize=(5,2.5))
# 	im = ax.pcolormesh(x, y, z, cmap=cmap)
# 	fig.colorbar(im, ax=ax, label="Average inter-prism \nerror [mm]")
# 	if(Speed == True):
# 		ax.set_xlabel("Linear speed [m/s]")
# 	else:
# 		ax.set_xlabel("Acceleration [m/s²]")
# 	ax.set_ylabel("Angular speed around \n Z axis [rad/s]")
# 	ax.set_facecolor("silver")
# 	plt.rc('font', size=10)
#
# 	fig.tight_layout()
# 	plt.show()
# 	if(save_fig):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# ###################################################################################################
# ###################################################################################################
# # Plot analyses of ptp
#
# # Function to plot the result of the effect on the noise on the X, Y, Z position, and roll, pitch, yaw angle in 6 subplots
# # Input:
# # - noise: list of the noise value for the x axis (m)
# # - statistic_x, statistic_y, statistic_z: uncertainty for each axis (m)
# # - statistic_roll, statistic_pitch, statistic_yaw: uncertainty for each angle (deg)
# # - save_fig: param if we want to save the figure in pdf
# # - name_file: name of the file to save the figure
# # - plot_figure: option to plot or not the figure
# def plot_results_analysis_ptp(noise, statistic_x, statistic_y, statistic_z, statistic_roll, statistic_pitch, statistic_yaw, save_fig, name_file, plot_figure):
# 	fig, axs = plt.subplots(2, 3, figsize=(6, 3), sharex=True)
# 	axs[0, 0].set_ylim([-0.3*1000,0.35*1000])
# 	axs[0, 0].plot(noise*1000, np.array(statistic_x)[:,0]*1000, color = 'goldenrod', linewidth=2.0)
# 	axs[0, 0].plot(noise*1000, np.array(statistic_x)[:,0]*1000-np.array(statistic_x)[:,1]*1000, color = 'goldenrod', alpha=0.3)
# 	axs[0, 0].plot(noise*1000, np.array(statistic_x)[:,0]*1000+np.array(statistic_x)[:,1]*1000, color = 'goldenrod', alpha=0.3)
# 	axs[0, 0].fill_between(noise*1000,np.array(statistic_x)[:,0]*1000-np.array(statistic_x)[:,1]*1000, np.array(statistic_x)[:,0]*1000+np.array(statistic_x)[:,1]*1000, color = 'goldenrod', alpha=0.3)
# 	#axs[0, 0].set_xlabel('Noise uncertainty (m)')
# 	axs[0, 0].set_ylabel('Bias X [mm]')
#
# 	axs[0, 1].plot(noise*1000, np.array(statistic_y)[:,0]*1000, color = 'darkblue', linewidth=2.0)
# 	axs[0, 1].plot(noise*1000, np.array(statistic_y)[:,0]*1000-np.array(statistic_y)[:,1]*1000, color = 'darkblue', alpha=0.3)
# 	axs[0, 1].plot(noise*1000, np.array(statistic_y)[:,0]*1000+np.array(statistic_y)[:,1]*1000, color = 'darkblue', alpha=0.3)
# 	axs[0, 1].fill_between(noise*1000,np.array(statistic_y)[:,0]*1000-np.array(statistic_y)[:,1]*1000, np.array(statistic_y)[:,0]*1000+np.array(statistic_y)[:,1]*1000, color = 'darkblue', alpha=0.3)
# 	#axs[0, 1].set_xlabel('Noise uncertainty (m)')
# 	axs[0, 1].set_ylabel('Bias Y [mm]')
# 	axs[0, 1].set_ylim([-0.3*1000,0.35*1000])
#
# 	axs[0, 2].plot(noise*1000, np.array(statistic_z)[:,0]*1000, color = 'sienna', linewidth=2.0)
# 	axs[0, 2].plot(noise*1000, np.array(statistic_z)[:,0]*1000-np.array(statistic_z)[:,1]*1000, color = 'sienna', alpha=0.3)
# 	axs[0, 2].plot(noise*1000, np.array(statistic_z)[:,0]*1000+np.array(statistic_z)[:,1]*1000, color = 'sienna', alpha=0.3)
# 	axs[0, 2].fill_between(noise*1000,np.array(statistic_z)[:,0]*1000-np.array(statistic_z)[:,1]*1000, np.array(statistic_z)[:,0]*1000+np.array(statistic_z)[:,1]*1000, color = 'sienna', alpha=0.3)
# 	#axs[0, 2].set_xlabel('Noise uncertainty (m)')
# 	axs[0, 2].set_ylabel('Bias Z [mm]')
# 	axs[0, 2].set_ylim([-0.3*1000,0.35*1000])
#
# 	axs[1, 0].plot(noise*1000, np.array(statistic_roll)[:,0]*np.pi/180, color = 'goldenrod', linewidth=2.0)
# 	axs[1, 0].plot(noise*1000, np.array(statistic_roll)[:,0]*np.pi/180-np.array(statistic_roll)[:,1]*np.pi/180, color = 'goldenrod', alpha=0.3)
# 	axs[1, 0].plot(noise*1000, np.array(statistic_roll)[:,0]*np.pi/180+np.array(statistic_roll)[:,1]*np.pi/180, color = 'goldenrod', alpha=0.3)
# 	axs[1, 0].fill_between(noise*1000,np.array(statistic_roll)[:,0]*np.pi/180-np.array(statistic_roll)[:,1]*np.pi/180, np.array(statistic_roll)[:,0]*np.pi/180+np.array(statistic_roll)[:,1]*np.pi/180, color = 'goldenrod', alpha=0.3)
# 	#axs[1, 0].set_xlabel('Noise uncertainty [m]')
# 	axs[1, 0].set_ylabel('Bias roll [rad]')
# 	#axs[1, 0].set_ylim([-0.8,0.8])
#
# 	axs[1, 1].plot(noise*1000, np.array(statistic_pitch)[:,0]*np.pi/180, color = 'darkblue', linewidth=2.0)
# 	axs[1, 1].plot(noise*1000, np.array(statistic_pitch)[:,0]*np.pi/180-np.array(statistic_pitch)[:,1]*np.pi/180, color = 'darkblue', alpha=0.3)
# 	axs[1, 1].plot(noise*1000, np.array(statistic_pitch)[:,0]*np.pi/180+np.array(statistic_pitch)[:,1]*np.pi/180, color = 'darkblue', alpha=0.3)
# 	axs[1, 1].fill_between(noise*1000,np.array(statistic_pitch)[:,0]*np.pi/180-np.array(statistic_pitch)[:,1]*np.pi/180, np.array(statistic_pitch)[:,0]*np.pi/180+np.array(statistic_pitch)[:,1]*np.pi/180, color = 'darkblue', alpha=0.3)
# 	axs[1, 1].set_xlabel('Noise uncertainty [mm]')
# 	axs[1, 1].set_ylabel('Bias pitch [rad]')
# 	#axs[1, 1].set_ylim([-0.8,0.8])
#
# 	axs[1, 2].plot(noise*1000, np.array(statistic_yaw)[:,0]*np.pi/180, color = 'sienna', linewidth=2.0)
# 	axs[1, 2].plot(noise*1000, np.array(statistic_yaw)[:,0]*np.pi/180-np.array(statistic_yaw)[:,1]*np.pi/180, color = 'sienna', alpha=0.3)
# 	axs[1, 2].plot(noise*1000, np.array(statistic_yaw)[:,0]*np.pi/180+np.array(statistic_yaw)[:,1]*np.pi/180, color = 'sienna', alpha=0.3)
# 	axs[1, 2].fill_between(noise*1000,np.array(statistic_yaw)[:,0]*np.pi/180-np.array(statistic_yaw)[:,1]*np.pi/180, np.array(statistic_yaw)[:,0]*np.pi/180+np.array(statistic_yaw)[:,1]*np.pi/180, color = 'sienna', alpha=0.3)
# 	#axs[1, 2].set_xlabel('Noise uncertainty [m]')
# 	axs[1, 2].set_ylabel('Bias yaw [rad]')
# 	#axs[1, 2].set_ylim([-0.8,0.8])
# 	plt.rc('font', size=10)
# 	fig.subplots_adjust(wspace=0.1, hspace=0)
#
# 	fig.tight_layout()
# 	if(plot_figure):
# 		plt.show()
# 	if(save_fig):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# ###################################################################################################
# ###################################################################################################
# # Plot GPS data
#
# # Function to plot the inter-GPS error in 2 subplot, 1: without RTK, 2: with RTK
# # Input:
# # - time_gps: list of timestamp RTK GPS (s)
# # - distance_gps: list of inter-gps distance RTK (m)
# # - time_gps_basic: list of timestamp without RTK GPS (s)
# # - distance_gps_basic: list of inter-gps distance without RTK (m)
# # - save_fig: param if we want to save the figure in pdf
# # - name_file: name of the file to save the figure
# def plot_gps_distance_multi(time_gps, distance_gps, time_gps_basic, distance_gps_basic, save_fig, name_file):
# 	fig = plt.figure(figsize=(6,4))
# 	ax = fig.add_subplot(211)
# 	plt.plot(np.array(time_gps_basic), abs(np.array(distance_gps_basic)-809.95/1000), 'r')
# 	#ax.set_xlabel('Time (s)')
# 	ax.set_ylabel('Error between GPS \n without RTK mode (m) \n')
#
# 	ax = fig.add_subplot(212)
# 	plt.plot(np.array(time_gps), abs(np.array(distance_gps)*1000-809.95), 'goldenrod')
# 	ax.set_xlabel('Time (s)')
# 	ax.set_ylabel('Error between GPS \nin RTK mode (mm)')
#
# 	fig.tight_layout()
# 	plt.show()
# 	if(save_fig):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# # Function to plot the inter-GPS error in 2 subplot, 1: inter-gps error, 2: histogram of the inter-gps error
# # Input:
# # - time_gps: list of timestamp RTK GPS (s)
# # - distance_gps: list of inter-gps distance RTK (m)
# # - dist_theoric: theoritical distance from both GPS (mm)
# # - save_fig: param if we want to save the figure in pdf
# # - name_file: name of the file to save the figure
# def plot_gps_distance(time_gps, distance_gps, dist_theoric, bin_number, save_fig, name_file):
# 	scale_x_min = min(distance_gps)-10
# 	scale_x_max = max(distance_gps)+10
# 	x = np.linspace(scale_x_min,scale_x_max,200) #for plot distribution
# 	mean = np.mean(np.array(distance_gps) - np.ones_like(distance_gps)*dist_theoric)*1000
# 	std = np.std(np.array(distance_gps)  - np.ones_like(distance_gps)*dist_theoric)*1000
#
# 	#spec = gridspec.GridSpec(ncols=2, nrows=1, width_ratios=[2, 1])
# 	fig, axs = plt.subplots(1, 2, figsize=(6, 2), gridspec_kw={'width_ratios': [2, 1]})
# 	#plt.plot(np.array(time_gps), abs(np.array(distance_gps)*1000-809.95), 'r')
# 	#axs[0] = fig.add_subplot(spec[0])
# 	axs[0].plot(np.array(time_gps)-np.ones_like(time_gps)*time_gps[0], abs(np.array(distance_gps)- np.ones_like(distance_gps)*dist_theoric)*1000, 'goldenrod')
#
# 	axs[0].set_xlabel('Time [s]')
# 	axs[0].set_ylabel('Error [mm]')
#
# 	#axs[1] = fig.add_subplot(spec[1])
# 	logbins = np.geomspace(np.min(abs(np.array(distance_gps)- np.ones_like(distance_gps)*dist_theoric)*1000),
# 						   np.max(abs(np.array(distance_gps)- np.ones_like(distance_gps)*dist_theoric)*1000), bin_number)
# 	axs[1] = sns.distplot(abs(np.array(distance_gps)- np.ones_like(distance_gps)*dist_theoric)*1000, color='goldenrod', bins=logbins, kde=False, hist_kws={'edgecolor': 'None', 'alpha': 0.7})
# 	#axs[1] = sns.distplot(abs(np.array(distance_gps)*1000-dist_theoric), kde=False, color='goldenrod', bins=20, hist_kws={'edgecolor':'None', 'alpha':0.7})
# 	axis_label = 'Error [mm]'
# 	axs[1].set_xlabel(axis_label)
# 	axs[1].set_ylabel('Counts')
# 	textmean = "Mean: " + str(round(mean, 5)) + "mm   " + "Std: " + str(round(std, 5)) + "mm   "
# 	fig.subplots_adjust(bottom=0.26, top=0.6)
# 	fig.text(0.35, 0.91, textmean, fontsize=10, transform=plt.gcf().transFigure)
# 	fig.tight_layout()
# 	#plt.title(textmean)
# 	plt.show()
# 	if(save_fig):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# # Function to plot the linear speed of a GPS and the speed of the robot
# # Input:
# # - GPS_front_utm_data: list of a GPS data
# # - speed: list of the linear speed of the robot
# def plot_linear_speed_gps_warthog(GPS_front_utm_data, speed):
# 	linear_speed_gps_utm = speed_gps(GPS_front_utm_data)
# 	fig = plt.figure(figsize=(8,6))
# 	ax = fig.add_subplot(111)
# 	plt.plot(np.array(linear_speed_gps_utm)[:,0], np.array(linear_speed_gps_utm)[:,1], 'r')
# 	plt.plot(np.array(speed)[:,0], np.array(speed)[:,1], 'b')
# 	plt.show()
#
# # Function to plot both GPS trajectory
# # Input:
# # - GPS_front_utm_data: list of front GPS data
# # - GPS_back_utm_data: list of back GPS data
# # - save_fig: param if we want to save the figure in pdf
# # - name_file: name of the file to save the figure
# def plot_gps_trajectory(GPS_front_utm_data, GPS_back_utm_data, plot_figure, save_fig, name_file):
# 	fig = plt.figure(figsize=(8,6))
# 	ax = fig.add_subplot(111)
# 	plt.plot(np.array(GPS_front_utm_data)[:,1], np.array(GPS_front_utm_data)[:,2], 'r')
# 	plt.plot(np.array(GPS_back_utm_data)[:,1], np.array(GPS_back_utm_data)[:,2], 'r')
# 	fig.tight_layout()
# 	ax.set_xlabel('Time (s)')
# 	ax.set_ylabel('Distance between GNSS (m)')
# 	if(plot_figure):
# 		plt.show()
# 	if(save_fig):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# # Function to plot the inter-GPS distance in one figure
# # Input:
# # - time_gps_basic: list of timestamp without RTK GPS (s)
# # - time_gps_RTK: list of timestamp RTK GPS (s)
# # - distance_gps_basic: list of inter-gps distance without RTK (m)
# # - distance_gps_RTK: list of inter-gps distance RTK (m)
# # - plot_figure: option to plot or not the figure
# # - save_fig: param if we want to save the figure in pdf
# # - name_file: name of the file to save the figure
# def subplot_gps_distance(time_gps_basic, time_gps_RTK, distance_gps_basic, distance_gps_RTK, plot_figure, save_fig, name_file):
# 	fig = plt.figure(figsize=(8,6))
# 	ax = fig.add_subplot(111)
# 	plt.plot(np.array(time_gps_basic), np.array(distance_gps_basic), 'r', label='Without RTK mode')
# 	plt.plot(np.array(time_gps_RTK), np.array(distance_gps_RTK), 'b', label='With RTK mode')
# 	ax.set_xlabel('Time (s)')
# 	ax.set_ylabel('Distance between GNSS without RTK mode (m)')
# 	ax.legend()
# 	if(plot_figure):
# 		plt.show()
# 	if(save_fig):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# ###################################################################################################
# ###################################################################################################
# # Plot GPS and theodolite data
#
# # Function to plot the inter-GPS error with the inter-prism error in 2 subplot, 1: both inter error over time, 2: histogram of both inter error
# # Input:
# # - interpolated_trajectories: list of the interpolated trajectories
# # - interpolated_time: list of the time for each interpolated trajectories
# # - time_gps: list of timestamp (s)
# # - distance_gps: list of inter-gps distance (m)
# # - save_fig: param if we want to save the figure in pdf
# # - name_file: name of the file to save the figure
# def plot_prism_error_and_gps(interpolated_trajectories, interpolated_time, time_gps, distance_gps, save_fig, name_file):
# 	time_value = []
# 	dist_error = []
# 	compteur = 0
# 	origin_time = 0
# 	double_origin_time = 0
# 	gps_error = []
# 	for i,j in zip(interpolated_trajectories, interpolated_time):
# 		for k in range(0,len(j)):
# 			if(compteur == 0):
# 				origin_time = j[k]
# 			#if(compteur == 7280):
# 			#	double_origin_time = time_value[compteur-1]
# 			#time_value.append(j[k] - origin_time + double_origin_time)
# 			time_value.append(j[k] - origin_time)
# 			compteur=compteur+1
# 			position_1 = i[0][0:3,k]
# 			position_2 = i[1][0:3,k]
# 			position_3 = i[2][0:3,k]
# 			dist_12_value = abs(np.linalg.norm(position_1-position_2)*1000-Dist_prism_12)
# 			dist_13_value = abs(np.linalg.norm(position_1-position_3)*1000-Dist_prism_13)
# 			dist_23_value = abs(np.linalg.norm(position_2-position_3)*1000-Dist_prism_23)
# 			dist_error.append(np.mean(np.array([dist_12_value, dist_13_value, dist_23_value])))
#
# 	for i in distance_gps:
# 		gps_error.append(abs(i*1000-809.95))
#
# 	time_value_arr = np.array(time_value)
# 	dist_error_arr = np.array(dist_error)
# 	begin_time = 0
# 	end_time = time_value_arr[-1]
# 	#begin_time = 80
# 	#end_time = 140
#
# 	mean_gps = np.mean(gps_error)
# 	std_gps = np.std(gps_error)
# 	mean_prism = np.mean(dist_error)
# 	std_prism = np.std(dist_error)
#
# 	fig, axs = plt.subplots(1, 2, figsize=(6, 2), gridspec_kw={'width_ratios': [2, 1]})
# 	axs[0].plot(time_value_arr, dist_error_arr, 'sienna', label="Inter-prism")
# 	axs[0].set_xlim([begin_time, end_time])
# 	axs[0].set_ylabel("Error [mm]")
# 	axs[0].plot(np.array(time_gps), abs(np.array(distance_gps)*1000-809.95), 'darkblue', label="Inter-GPS", alpha=0.7)
# 	axs[0].set_xlabel('Time [s]')
# 	#axs[0].legend()
#
# 	distance_list = [distance_gps, dist_error]
# 	group_labels = ['Inter-GPS', 'Inter-rism']
# 	sns.distplot(dist_error, kde=False, color='sienna', bins=[0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34], hist_kws={'edgecolor':'None', 'alpha':1}, ax=axs[1])
# 	axs[1].set_xlim([0, 35])
# 	ax2=axs[1].twinx()
# 	sns.distplot(gps_error, kde=False, color='darkblue', bins=[0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34], hist_kws={'edgecolor':'None', 'alpha':0.7}, ax=ax2)
# 	ax2.set_ylabel("Counts prism")
# 	#ax2.set_xlim([begin_time, end_time])
# 	#ax2.set_ylim([0, 0.6])
# 	align_yaxis(axs[1], 0, ax2, 0)
# 	#axs[1].legend()
# 	#ax2.legend()
# 	#plt.rc('font', size=12)
#
# 	axis_label = 'Error [mm]'#,\n $\mu_{GPS}$ = ' + str(round(mean_gps, 1)) + 'mm, $\sigma_{GPS}$ = ' + str(round(std_gps, 1)) + 'mm\n $\mu_{Prism}$ = ' + str(round(mean_prism, 1)) + 'mm, $\sigma_{Prism}$ = ' + str(round(std_prism, 1)) + 'mm'
# 	axs[1].set_xlabel(axis_label)
# 	axs[1].set_ylabel("Counts GPS")
#
# 	handles, labels = axs[0].get_legend_handles_labels()
# 	fig.legend(handles, labels, bbox_to_anchor=(0, 1, 1, 0), loc='upper center', ncol=2)
# 	fig.subplots_adjust(top=0.2, bottom=0,  wspace=0.3)
#
# 	fig.tight_layout()
# 	plt.show()
# 	if(save_fig):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# # Function to plot the inter-GPS error with the inter-prism error in 3 subplot, 1: both inter error over time in interval 1,
# # 2: both inter error over time in interval 2, 3: histogram of both inter error
# # Input:
# # - interpolated_trajectories: list of the interpolated trajectories
# # - interpolated_time: list of the time for each interpolated trajectories
# # - time_gps: list of timestamp (s)
# # - distance_gps: list of inter-gps distance (m)
# # - save_fig: param if we want to save the figure in pdf
# # - name_file: name of the file to save the figure
# # - interval: list of 2 1x2 array of the interval of time to sort the data, [0]: begin and [1] end
# def subplot_prism_error_and_gps(interpolated_trajectories, interpolated_time, time_gps, distance_gps, save_fig, name_file, interval):
# 	time_value = []
# 	dist_error = []
# 	compteur = 0
# 	origin_time = 0
# 	double_origin_time = 0
# 	for i,j in zip(interpolated_trajectories, interpolated_time):
# 		for k in range(0,len(j)):
# 			if(compteur == 0):
# 				origin_time = j[k]
# 			#if(compteur == 7280):
# 			#	double_origin_time = time_value[compteur-1]
# 			#time_value.append(j[k] - origin_time + double_origin_time)
# 			time_value.append(j[k] - origin_time)
# 			compteur=compteur+1
# 			position_1 = i[0][0:3,k]
# 			position_2 = i[1][0:3,k]
# 			position_3 = i[2][0:3,k]
# 			dist_12_value = abs(np.linalg.norm(position_1-position_2)*1000-Dist_prism_12)
# 			dist_13_value = abs(np.linalg.norm(position_1-position_3)*1000-Dist_prism_13)
# 			dist_23_value = abs(np.linalg.norm(position_2-position_3)*1000-Dist_prism_23)
# 			dist_error.append(np.mean(np.array([dist_12_value, dist_13_value, dist_23_value])))
# 			#dist_error.append(dist_12_value)
# 			#dist_error.append(dist_13_value)
# 			#dist_error.append(dist_23_value)
#
# 	#print(np.mean(dist_error))
# 	#print(np.std(dist_error))
#
# 	time_value_arr = np.array(time_value)
# 	dist_error_arr = np.array(dist_error)
# 	begin_time_1 = interval[0][0]
# 	end_time_1 = interval[0][1]
# 	begin_time_2 = interval[1][0]
# 	end_time_2 = interval[1][1]
#
# 	mean_gps = np.mean(abs(np.array(distance_gps)*1000-809.95))
# 	std_gps = np.std(abs(np.array(distance_gps)*1000-809.95))
# 	mean_prism = np.mean(dist_error)
# 	std_prism = np.std(dist_error)
#
# 	print(mean_gps, std_gps)
# 	print(mean_prism, std_prism)
#
# 	gs1 = GridSpec(1, 1)
# 	gs1.update(left=0.12, right=0.35, wspace=0.05, bottom = 0.24)
#
# 	fig = plt.figure(figsize=(6, 2))
# 	#fig, axs = plt.subplots(1, 3, figsize=(6, 2), gridspec_kw={'width_ratios': [1, 1, 1]}, constrained_layout = True)
# 	#fig, axs = plt.subplots(1, 3, figsize=(6, 2))
#
# 	axs0 = plt.subplot(gs1[:, -1])
# 	axs0.plot(time_value_arr, dist_error_arr, 'sienna', label="Inter-prism")
# 	axs0.plot(np.array(time_gps), abs(np.array(distance_gps)*1000-809.95), 'darkblue', label="Inter-GPS", alpha=0.7)
# 	axs0.set_xlim([begin_time_1, end_time_1])
# 	axs0.set_ylim([0, 600])
# 	axs0.set_xlabel('Time [s]')
# 	axs0.set_ylabel('Error [mm]')
# 	axs0.set_yscale('symlog')
#
# 	gs2 = GridSpec(1, 2)
# 	gs2.update(left=0.37, right=0.9, wspace=0.47, bottom = 0.24)
#
# 	axs1 = plt.subplot(gs2[:, :-1])
# 	axs1.plot(time_value_arr, dist_error_arr, 'sienna', label="Inter-prism")
# 	axs1.plot(np.array(time_gps), abs(np.array(distance_gps)*1000-809.95), 'darkblue', label="Inter-GPS", alpha=0.7)
# 	axs1.set_xlim([begin_time_2, end_time_2])
# 	axs1.set_ylim([0, 600])
# 	axs1.set_xlabel('Time [s]')
# 	axs1.set_yscale('symlog')
# 	axs1.axes.yaxis.set_visible(False)
#
# 	axs2 = plt.subplot(gs2[:, -1])
# 	distance_list = [distance_gps, dist_error]
# 	group_labels = ['Inter-GPS', 'Inter-rism']
# 	logbins = np.geomspace(np.min(dist_error), np.max(dist_error), 10)
# 	sns.distplot(dist_error, kde=False, color='sienna', bins=logbins, hist_kws={'edgecolor':'None', 'alpha':1}, ax=axs2)
# 	#axs[2].set_xlim([0, 25])
# 	ax22=axs2.twinx()
# 	logbins = np.geomspace(np.min(abs(np.array(distance_gps)*1000-809.95)), np.max(abs(np.array(distance_gps)*1000-809.95)), 20)
# 	sns.distplot(abs(np.array(distance_gps)*1000-809.95), kde=False, color='darkblue', bins=logbins, hist_kws={'edgecolor':'None', 'alpha':0.7}, ax=ax22)
# 	ax22.set_ylabel("Counts prism")
# 	#ax2.set_xlim([begin_time, end_time])
# 	#axs[2].set_ylim([0, 200000])
# 	#ax2.set_ylim([0, 200000])
# 	align_yaxis(axs2, 0, ax22, 0)
# 	axs2.set_xscale('symlog')
# 	axs2.set_yscale('symlog')
# 	ax22.set_xscale('symlog')
# 	ax22.set_yscale('symlog')
# 	#axs[1].legend()
# 	#ax2.legend()
# 	#plt.rc('font', size=12)
#
# 	axis_label = 'Error [mm]'
# 	axs2.set_xlabel(axis_label)
# 	axs2.set_ylabel("Counts GPS")
#
# 	#fig.subplots_adjust(wspace=0.3, bottom = 0.26, top = 0.91)
# 	lines = []
# 	labels = []
#
# 	axLine, axLabel = axs0.get_legend_handles_labels()
# 	lines.extend(axLine)
# 	labels.extend(axLabel)
# 	#fig.legend(lines, labels, bbox_to_anchor=(0, 1, 1, 0), loc='upper center', ncol=2)
# 	#fig.legend(handles=[h0, h1, h2], labels=[l0, l1, l2], bbox_to_anchor=(0, 1, 1, 0), loc='upper center')
# 	#handles, labels = ax.get_legend_handles_labels()
# 	#fig.legend(handles, labels, bbox_to_anchor=(0, 1, 1, 0), loc='upper center', ncol=3)
#
# 	#fig.subplots_adjust(wspace=0.1, hspace=0)
#
# 	#fig.tight_layout()
# 	plt.show()
# 	if(save_fig):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# # Function to plot the inter-GPS error with the inter-prism error in 3 subplot, 1: both inter error over time in interval 1,
# # 2: both inter error over time in interval 2, 3: histogram of both inter error
# # Input:
# # - time: list of the interpolated trajectories
# # - error_distance: list of the time for each interpolated trajectories
# # - save_fig: param if we want to save the figure in pdf
# # - name_file: name of the file to save the figure
# # - interval: list of 3x1x2 array of the interval of time to sort the data, [0]: begin and [1] end
# def subplot_prisms_error(time, error_distance, save_fig, name_file):
#
# 	origin_time = time[0]
# 	total_samples = len(time)
# 	time_value_arr = np.array(time)-origin_time
# 	mean_prism12 = np.mean(np.array(error_distance).T[0])
# 	std_prism12 = np.std(np.array(error_distance).T[0])
# 	mean_prism13 = np.mean(np.array(error_distance).T[1])
# 	std_prism13 = np.std(np.array(error_distance).T[1])
# 	mean_prism23 = np.mean(np.array(error_distance).T[2])
# 	std_prism23 = np.std(np.array(error_distance).T[2])
# 	print(total_samples)
# 	print(mean_prism12, mean_prism13, mean_prism23)
# 	print(std_prism12, std_prism13, std_prism23)
#
# 	number_bins = 20
#
# 	gs1 = GridSpec(1, 1)
# 	gs1.update(left=0.12, right=0.35, wspace=0.05, bottom=0.24)
#
# 	fig = plt.figure(figsize=(8, 2))
# 	# fig, axs = plt.subplots(1, 3, figsize=(6, 2), gridspec_kw={'width_ratios': [1, 1, 1]}, constrained_layout = True)
# 	# fig, axs = plt.subplots(1, 3, figsize=(6, 2))
#
# 	axs0 = plt.subplot(gs1[:, -1])
# 	axs0.plot(time_value_arr, np.array(error_distance).T[0], 'sienna', label="E12", alpha=0.7)
# 	axs0.plot(time_value_arr, np.array(error_distance).T[1], 'darkblue', label="E13", alpha=0.7)
# 	axs0.plot(time_value_arr, np.array(error_distance).T[2], 'goldenrod', label="E23", alpha=0.7)
# 	#axs0.set_ylim([0, 600])
# 	axs0.set_xlabel('Time [s]')
# 	axs0.set_ylabel('Error [mm]')
# 	axs0.set_yscale('symlog')
# 	axs0.yaxis.set_major_formatter(FormatStrFormatter('% 1.2f'))
#
# 	gs2 = GridSpec(1, 3)
# 	gs2.update(left=0.43, right=0.9, wspace=0.47, bottom=0.24)
#
# 	axs1 = plt.subplot(gs2[:, 0])
# 	logbins = np.geomspace(np.min(np.array(error_distance).T[0]), np.max(np.array(error_distance).T[0]), number_bins)
# 	sns.distplot(np.array(error_distance).T[0], kde=False, color='sienna', bins=logbins, hist_kws={'edgecolor': 'None', 'alpha': 0.7},
# 				 ax=axs1)
# 	axs1.set_ylabel("Counts")
#
# 	axs2 = plt.subplot(gs2[:, 1])
# 	logbins = np.geomspace(np.min(np.array(error_distance).T[1]), np.max(np.array(error_distance).T[1]), number_bins)
# 	sns.distplot(np.array(error_distance).T[1], kde=False, color='darkblue', bins=logbins,
# 				 hist_kws={'edgecolor': 'None', 'alpha': 0.7}, ax=axs2)
# 	axs2.set_xlabel("Error [mm]")
#
# 	axs3 = plt.subplot(gs2[:, -1])
# 	logbins = np.geomspace(np.min(np.array(error_distance).T[2]), np.max(np.array(error_distance).T[2]), number_bins)
# 	sns.distplot(np.array(error_distance).T[2], kde=False, color='goldenrod', bins=logbins, hist_kws={'edgecolor': 'None', 'alpha': 0.7},
# 				 ax=axs3)
# 	# axs[2].set_xlim([0, 25])
# 	# ax2.set_xlim([begin_time, end_time])
# 	# axs[2].set_ylim([0, 200000])
# 	# ax2.set_ylim([0, 200000])
# 	#align_yaxis(axs2, 0, ax22, 0)
# 	axs1.set_xscale('symlog')
# 	axs1.set_yscale('symlog')
# 	axs2.set_xscale('symlog')
# 	axs2.set_yscale('symlog')
# 	axs3.set_xscale('symlog')
# 	axs3.set_yscale('symlog')
#
# 	axs1.xaxis.set_major_formatter(FormatStrFormatter('% 1.2f'))
# 	axs2.xaxis.set_major_formatter(FormatStrFormatter('% 1.2f'))
# 	axs3.xaxis.set_major_formatter(FormatStrFormatter('% 1.2f'))
# 	# axs[1].legend()
# 	# ax2.legend()
# 	# plt.rc('font', size=12)
#
# 	fig.subplots_adjust(wspace=0.3, bottom = 0.26, top = 0.7)
# 	#lines = []
# 	#labels = []
# 	#axLine, axLabel = axs0.get_legend_handles_labels()
# 	#lines.extend(axLine)
# 	#labels.extend(axLabel)
# 	# fig.legend(lines, labels, bbox_to_anchor=(0, 1, 1, 0), loc='upper center', ncol=2)
# 	# fig.legend(handles=[h0, h1, h2], labels=[l0, l1, l2], bbox_to_anchor=(0, 1, 1, 0), loc='upper center')
# 	#handles, labels = ax.get_legend_handles_labels()
#
# 	brown_patch = mpatches.Patch(color='sienna', label='E12')
# 	blue_patch = mpatches.Patch(color='darkblue', label='E13')
# 	yellow_patch = mpatches.Patch(color='goldenrod', label='E23')
# 	handles = [brown_patch, blue_patch, yellow_patch]
# 	labels = ['Inter-prism 12', 'Inter-prism 13', 'Inter-prism 23']
# 	fig.legend(handles, labels, bbox_to_anchor=(0, 1, 1, 0), loc='upper center', ncol=3)
#
# 	textmean = "Mean: " + str(round(mean_prism12,6)) + "mm   " + str(round(mean_prism13,6)) + "mm   " + str(round(mean_prism23,6)) + "mm   "
# 	plt.text(0.15, 0.75, textmean, fontsize=10, transform=plt.gcf().transFigure)
# 	textstd = "Std: " + str(round(std_prism12, 6)) + "mm   " + str(round(std_prism13, 6)) + "mm   " + str(round(std_prism23, 6)) + "mm   "
# 	plt.text(0.55, 0.75, textstd, fontsize=10, transform=plt.gcf().transFigure)
#
# 	#fig.subplots_adjust(wspace=0.1, hspace=0)
# 	#fig.tight_layout()
# 	plt.show()
# 	if (save_fig):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# def subplot_prisms_error_gt(time1, time2, time3, error_distance1, error_distance2, error_distance3, save_fig, name_file):
#
# 	time_value_arr1 = np.array(time1)
# 	time_value_arr2 = np.array(time2)
# 	time_value_arr3 = np.array(time3)
# 	origin = min(time_value_arr1[0],time_value_arr2[0],time_value_arr3[0])
# 	mean_prism1 = np.mean(np.array(error_distance1))
# 	std_prism1 = np.std(np.array(error_distance1))
# 	mean_prism2 = np.mean(np.array(error_distance2))
# 	std_prism2 = np.std(np.array(error_distance2))
# 	mean_prism3 = np.mean(np.array(error_distance3))
# 	std_prism3 = np.std(np.array(error_distance3))
# 	print(mean_prism1, mean_prism2, mean_prism3)
# 	print(std_prism1, std_prism2, std_prism3)
#
# 	number_bins = 20
#
# 	gs1 = GridSpec(1, 1)
# 	gs1.update(left=0.12, right=0.35, wspace=0.05, bottom=0.24)
#
# 	fig = plt.figure(figsize=(8, 2))
#
# 	axs0 = plt.subplot(gs1[:, -1])
# 	axs0.plot(time_value_arr1-np.ones_like(time_value_arr1)*origin, np.array(error_distance1), 'sienna', label="E1", alpha=0.7)
# 	axs0.plot(time_value_arr2-np.ones_like(time_value_arr2)*origin, np.array(error_distance2), 'darkblue', label="E2", alpha=0.7)
# 	axs0.plot(time_value_arr3-np.ones_like(time_value_arr3)*origin, np.array(error_distance3), 'goldenrod', label="E3", alpha=0.7)
# 	#axs0.set_ylim([0, 100])
# 	axs0.set_xlabel('Time [s]')
# 	axs0.set_ylabel('Error [mm]')
# 	axs0.set_yscale('symlog')
# 	axs0.yaxis.set_major_formatter(FormatStrFormatter('% 1.2f'))
#
# 	gs2 = GridSpec(1, 3)
# 	gs2.update(left=0.43, right=0.9, wspace=0.47, bottom=0.24)
#
# 	axs1 = plt.subplot(gs2[:, 0])
# 	logbins = np.geomspace(np.min(np.array(error_distance1)), np.max(np.array(error_distance1)), number_bins)
# 	sns.distplot(np.array(error_distance1), kde=False, color='sienna', bins=logbins,
# 				 hist_kws={'edgecolor': 'None', 'alpha': 0.7},
# 				 ax=axs1)
# 	axs1.set_ylabel("Counts")
#
# 	axs2 = plt.subplot(gs2[:, 1])
# 	logbins = np.geomspace(np.min(np.array(error_distance2)), np.max(np.array(error_distance2)), number_bins)
# 	sns.distplot(np.array(error_distance2), kde=False, color='darkblue', bins=logbins,
# 				 hist_kws={'edgecolor': 'None', 'alpha': 0.7}, ax=axs2)
# 	axs2.set_xlabel("Error [mm]")
#
# 	axs3 = plt.subplot(gs2[:, -1])
# 	logbins = np.geomspace(np.min(np.array(error_distance3)), np.max(np.array(error_distance3)), number_bins)
# 	sns.distplot(np.array(error_distance3), kde=False, color='goldenrod', bins=logbins,
# 				 hist_kws={'edgecolor': 'None', 'alpha': 0.7},
# 				 ax=axs3)
#
# 	axs1.set_xscale('symlog')
# 	axs1.set_yscale('symlog')
# 	axs2.set_xscale('symlog')
# 	axs2.set_yscale('symlog')
# 	axs3.set_xscale('symlog')
# 	axs3.set_yscale('symlog')
# 	#axs1.set_xlim([-0.1, 10])
# 	#axs2.set_xlim([-0.1, 10])
# 	#axs3.set_xlim([-0.1, 10])
#
# 	axs1.xaxis.set_major_formatter(FormatStrFormatter('% 1.0f'))
# 	axs2.xaxis.set_major_formatter(FormatStrFormatter('% 1.0f'))
# 	axs3.xaxis.set_major_formatter(FormatStrFormatter('% 1.0f'))
#
# 	fig.subplots_adjust(wspace=0.3, bottom=0.26, top=0.7)
#
# 	brown_patch = mpatches.Patch(color='sienna', label='E12')
# 	blue_patch = mpatches.Patch(color='darkblue', label='E13')
# 	yellow_patch = mpatches.Patch(color='goldenrod', label='E23')
# 	handles = [brown_patch, blue_patch, yellow_patch]
# 	labels = ['Inter-Prism 1-2', 'Inter-Prism 1-3', 'Inter-Prism 2-3']
# 	fig.legend(handles, labels, bbox_to_anchor=(0, 1, 1, 0), loc='upper center', ncol=3)
#
# 	textmean = "Mean: " + str(round(mean_prism1, 2)) + "mm   " + str(round(mean_prism2, 2)) + "mm   " + str(
# 		round(mean_prism3, 2)) + "mm   "
# 	plt.text(0.15, 0.75, textmean, fontsize=10, transform=plt.gcf().transFigure)
# 	textstd = "Std: " + str(round(std_prism1, 2)) + "mm   " + str(round(std_prism2, 2)) + "mm   " + str(
# 		round(std_prism3, 2)) + "mm   "
# 	plt.text(0.55, 0.75, textstd, fontsize=10, transform=plt.gcf().transFigure)
#
# 	# fig.subplots_adjust(wspace=0.1, hspace=0)
# 	# fig.tight_layout()
# 	plt.show()
# 	if (save_fig):
# 		fig.savefig(name_file, bbox_inches='tight')
#
# def subplot_gps_error_gt(time1, time2, time3, error_distance1, error_distance2, error_distance3, save_fig, name_file):
#
# 	time_value_arr1 = np.array(time1)
# 	time_value_arr2 = np.array(time2)
# 	time_value_arr3 = np.array(time3)
# 	origin = min(time_value_arr1[0],time_value_arr2[0],time_value_arr3[0])
# 	mean_prism1 = np.mean(np.array(error_distance1))
# 	std_prism1 = np.std(np.array(error_distance1))
# 	mean_prism2 = np.mean(np.array(error_distance2))
# 	std_prism2 = np.std(np.array(error_distance2))
# 	mean_prism3 = np.mean(np.array(error_distance3))
# 	std_prism3 = np.std(np.array(error_distance3))
# 	print(mean_prism1, mean_prism2, mean_prism3)
# 	print(std_prism1, std_prism2, std_prism3)
#
# 	number_bins = 20
#
# 	gs1 = GridSpec(1, 1)
# 	gs1.update(left=0.12, right=0.35, wspace=0.05, bottom=0.24)
#
# 	fig = plt.figure(figsize=(8, 2))
#
# 	axs0 = plt.subplot(gs1[:, -1])
# 	axs0.plot(time_value_arr1-np.ones_like(time_value_arr1)*origin, np.array(error_distance1), 'sienna', label="E1", alpha=0.7)
# 	axs0.plot(time_value_arr2-np.ones_like(time_value_arr2)*origin, np.array(error_distance2), 'darkblue', label="E2", alpha=0.7)
# 	axs0.plot(time_value_arr3-np.ones_like(time_value_arr3)*origin, np.array(error_distance3), 'goldenrod', label="E3", alpha=0.7)
# 	#axs0.set_ylim([0, 100])
# 	axs0.set_xlabel('Time [s]')
# 	axs0.set_ylabel('Error [mm]')
# 	axs0.set_yscale('symlog')
# 	axs0.yaxis.set_major_formatter(FormatStrFormatter('% 1.2f'))
#
# 	gs2 = GridSpec(1, 3)
# 	gs2.update(left=0.43, right=0.9, wspace=0.47, bottom=0.24)
#
# 	axs1 = plt.subplot(gs2[:, 0])
# 	logbins = np.geomspace(np.min(np.array(error_distance1)), np.max(np.array(error_distance1)), number_bins)
# 	sns.distplot(np.array(error_distance1), kde=False, color='sienna', bins=logbins,
# 				 hist_kws={'edgecolor': 'None', 'alpha': 0.7},
# 				 ax=axs1)
# 	axs1.set_ylabel("Counts")
#
# 	axs2 = plt.subplot(gs2[:, 1])
# 	logbins = np.geomspace(np.min(np.array(error_distance2)), np.max(np.array(error_distance2)), number_bins)
# 	sns.distplot(np.array(error_distance2), kde=False, color='darkblue', bins=logbins,
# 				 hist_kws={'edgecolor': 'None', 'alpha': 0.7}, ax=axs2)
# 	axs2.set_xlabel("Error [mm]")
#
# 	axs3 = plt.subplot(gs2[:, -1])
# 	logbins = np.geomspace(np.min(np.array(error_distance3)), np.max(np.array(error_distance3)), number_bins)
# 	sns.distplot(np.array(error_distance3), kde=False, color='goldenrod', bins=logbins,
# 				 hist_kws={'edgecolor': 'None', 'alpha': 0.7},
# 				 ax=axs3)
#
# 	axs1.set_xscale('symlog')
# 	axs1.set_yscale('symlog')
# 	axs2.set_xscale('symlog')
# 	axs2.set_yscale('symlog')
# 	axs3.set_xscale('symlog')
# 	axs3.set_yscale('symlog')
# 	#axs1.set_xlim([-0.1, 10])
# 	#axs2.set_xlim([-0.1, 10])
# 	#axs3.set_xlim([-0.1, 10])
#
# 	axs1.xaxis.set_major_formatter(FormatStrFormatter('% 1.0f'))
# 	axs2.xaxis.set_major_formatter(FormatStrFormatter('% 1.0f'))
# 	axs3.xaxis.set_major_formatter(FormatStrFormatter('% 1.0f'))
#
# 	fig.subplots_adjust(wspace=0.3, bottom=0.26, top=0.7)
#
# 	brown_patch = mpatches.Patch(color='sienna', label='E12')
# 	blue_patch = mpatches.Patch(color='darkblue', label='E13')
# 	yellow_patch = mpatches.Patch(color='goldenrod', label='E23')
# 	handles = [brown_patch, blue_patch, yellow_patch]
# 	labels = ['Inter-GPS 1-2', 'Inter-GPS 1-3', 'Inter-GPS 2-3']
# 	fig.legend(handles, labels, bbox_to_anchor=(0, 1, 1, 0), loc='upper center', ncol=3)
#
# 	textmean = "Mean: " + str(round(mean_prism1, 2)) + "mm   " + str(round(mean_prism2, 2)) + "mm   " + str(
# 		round(mean_prism3, 2)) + "mm   "
# 	plt.text(0.15, 0.75, textmean, fontsize=10, transform=plt.gcf().transFigure)
# 	textstd = "Std: " + str(round(std_prism1, 2)) + "mm   " + str(round(std_prism2, 2)) + "mm   " + str(
# 		round(std_prism3, 2)) + "mm   "
# 	plt.text(0.55, 0.75, textstd, fontsize=10, transform=plt.gcf().transFigure)
#
# 	# fig.subplots_adjust(wspace=0.1, hspace=0)
# 	# fig.tight_layout()
# 	plt.show()
# 	if (save_fig):
# 		fig.savefig(name_file, bbox_inches='tight')