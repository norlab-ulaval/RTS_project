import numpy as np

import scripts.theodolite_utils as theodo_u

## Path of main data files
paths = [
    "../data/20220224/",  # 1 done Instrument, time synch
    "../data/20220307/",  # 2 done Instrument, time synch
    "../data/20220312/",  # 3 done Instrument, time synch
    "../data/20220314/",  # 4 done Instrument, time synch
    "../data/20220316/",  # 5 done Instrument, time synch
    "../data/20220331-1/",  # 6 done Instrument, time synch
    "../data/20220331-2/",  # 7 done Instrument, time synch
    "../data/20220513-1/",  # 10 done Instrument, time synch
    "../data/20220513-2/",  # 11 done Instrument, time synch
    "../data/20220513-3/",  # 12 done Instrument, time synch
    "../data/20220513-4/",  # 13 done Instrument, time synch
    "../data/20220513-5/",  # 14 done Instrument, time synch
    "../data/20220513-6/",  # 15 done Instrument, time synch
    "../data/20220525-1/",  # 16 done Instrument, time synch
    "../data/20220525-2/",  # 17 done Instrument, time synch
    "../data/20220622-1/",  # 18 done Instrument, time synch
    "../data/20220622-2/",  # 19 done Instrument, time synch
    "../data/20220630-1/",  # 20 done Instrument, time synch
    "../data/20220630-2/",  # 21 done Instrument, time synch
    "../data/20220711-1/",  # 22 done Instrument, time synch
    "../data/20220711-2/",  # 23 done Instrument, time synch
    "../data/20220715-1/",  # 24 done Instrument, time synch
    "../data/20220715-2/",  # 25 done Instrument, time synch
    "../data/20220715-3/",  # 26 done Instrument, time synch
    "../data/20220715-4/",  # 27 done Instrument, time synch
    "../data/20221103-1/",  # 28 done Instrument, time synch
    "../data/20221103-2/",  # 29 done Instrument, time synch
    "../data/20221103-3/",  # 30 done Instrument, time synch
    "../data/20221116-1/",  # 35 done Instrument, time synch
    "../data/20221123/",  # 36 done Instrument, time synch
    "../data/20221124/",  # 37 done Instrument, time synch
    "../data/20221129-1/",  # 38 done Instrument, time synch
    "../data/20221129-2/",  # 39 done Instrument, time synch
    "../data/20221129-3/",  # 40 done Instrument, time synch
    "../data/20221129-4/",  # 41 done Instrument, time synch
    "../data/20221129-5/",  # 42 done Instrument, time synch
    "../data/20221205-1/",  # 43 done Instrument, time synch
    "../data/20221205-2/",  # 44 done Instrument, time synch
    "../data/20221205-3/",  # 45 done Instrument, time synch
    "../data/20220427-1/",  # 8 done Instrument, time synch
    "../data/20220427-2/",  # 9 done Instrument, time synch
    "../data/20221109-1/",  # 31 done Instrument, time synch
    "../data/20221109-2/",  # 32 done Instrument, time synch
    "../data/20221109-3/",  # 33 done Instrument, time synch
    "../data/20221110/",  # 34 done Instrument, atmospheric
]

weather_list = [
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
]

## Choice of noise model
# Tilt noise, Atmospheric correction, extrinsic calibration, Time synchronization, Weather corrections # 1: Activate, 0:Disable
model_chosen = [1, 0, 0, 0, 0]

#############################################################################################################################################################
#############################################################################################################################################################
type_file_input = "CSV"  ## CSV or ROSBAG

## Parameters for pre-processing pipeline
parameters = [
    [
        1,
        2,
        1,
        1,
        3,
        2,
    ],  # 1. Apply filtering or not (Module 1), 2-3-4. Parameters tau_r, tau_a, tau_e (Module 1), 5. Parameter tau_s (Module 2), 6 Parameter tau_l (Module 4).
]

# Reading sensor data
# Sensor = "Robosense_32"
Sensor = "Fake"  # GNSS or Robosense_32 or Fake
# path_sensor_file = path+"ICP/icp_odom.csv"
rate_fake = 5  # Hz
Sensor_fake = "GNSS"
path_sensor_file = "gps_data/" + "gps1.txt"
path_sensor_file_synch_time = (
    "gps_data/delay_synchronization_GNSS_1.txt"  # If applied to GNSS
)
Gps_reference_chosen = 1  # 1: front, 2: back, 3: middle   # Only for GNSS

# Limit acceptable for ground truth error
limit_taken_into_accout = False
limit_dist = 100  # Need to check the impact
limit_uncertainty = 100  # Limit of eigenvalues taken into account

## Parameters about noise for MC
## Number sample MC
num_samples = 1000
## Range noise
random_noise_range = [
    0,
    0.004 / 2,
    2,
]  ## Mean, sigma, ppm,  4mm + 2ppm (2 sigma)  ISO17123-3
## Angles noise
random_noise_angle = [
    0,
    0.000024241 / 5 * 4 / 2,
]  # Mean, sigma, 5"=0.000024241 precision datasheet  (2 sigma)  ISO17123-3
## Tilt compensator noise
random_noise_tilt = [
    0,
    0.000002424 / 2,
]  # Mean, sigma, 0.5"=0.000002424 precision datasheet  (2 sigma)  ISO17123-3
## Weather conditions
weather_data_path = "../data/weather_2022/"
data_weather_quebec = np.array(
    theodo_u.read_weather_data(weather_data_path + "Quebec/data_sorted_2022.txt")
)
data_weather_fm = np.array(
    theodo_u.read_weather_data(
        weather_data_path + "Montmorency_forest/data_sorted_2022.txt"
    )
)
## Time synchronization
time_error_synch_mean = 1.157 * 10 ** (-3)  # Mean time error [s]
time_error_synch_std = 0.815 * 10 ** (-3)  # Std time error [s]

num_samples_MC_sensor = 100  ## Number sample for uncertainty propagation to the sensor
save_MC_inteprolated = True  ## Save interpolation done with STEAM
want_sensor_pose = False  ## Compute sensor uncertainty
save_MC_sensor = False  ## Save sensor pose with uncertainty
