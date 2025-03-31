#pragma once
#include <string>

const std::string ESTIMATES_FILE = "src/isam2/data/current_estimates.txt";
const bool EUFS_SIM_SETTINGS = false;
/* Bearing and range error
* Corresponds to the usage of the BearingRangeFactor we are using
*
* Source: https://chem.libretexts.org/Bookshelves/Analytical_Chemistry/
*         Supplemental_Modules_(Analytical_Chemistry)/Quantifying_Nature/
*         Significant_Digits/Propagation_of_Error
* Source: User manual for the AT128 Hesai LiDAR
* 
* bearing error in radians
* Calculation for error per radian: 
* We use atan2(y, x) and we know that z = atan(u) has derivative 1/(1+u^2) 
* std.dev_{u} = sqrt(0.03^2 + 0.03^2) = 0.0424
* std.dev_{z} = (1/(1+1^2))^2 * std.dev_{u}^2 = 1/4 * 0.0424^2 = 0.00045
* 
* Source: User manual for the AT128 Hesai LiDAR
* range error in meters
* std.dev_{u} = 0.0424 -> changed to 0.03
* 
*/
const double BEARING_STD_DEV = 0.00045;
const double RANGE_STD_DEV = 0.03;


/* Go from 1 pose to another pose
* Source: https://www.movella.com/products/sensor-modules/xsens-mti-680g-rtk-gnss-ins
*
* x error in meters (must be based on velocity error)
* y error in meters (must be based on velocity error)
* velocity error = 0.05 m/s RMS
* Calculation for the error per meter: 1m = (1 +- 0.05 m/s) * (1 +- 1e-9 s)
* std. dev = 1 * sqrt((0.05/1)^2 + (1e-9/1)^2) = 0.05 
*
* yaw error was 0.5 degrees RMS = std dev (Must convert to radians)
* changed to 0.009 degrees (Must convert to radians)
*/

const double IMU_X_STD_DEV = 0.22;
const double IMU_Y_STD_DEV = 0.22;
const double IMU_HEADING_STD_DEV = 0.00015707963;



/* GPS noise model 
* Use the covariances from positionlla
*
* Covariance matrix diagonal elements represent variances
* Variance = (std.dev)^2 meaning:
* Variance = 0.2 => std.dev = sqrt(0.2) = 0.45
* 
* However positionlla already accounts for the covariance
* so we are using std.dev = 0.01
* 
*/
const double GPS_X_STD_DEV = 0.01;
const double GPS_Y_STD_DEV = 0.01;


/***** EUFS_SIM ******/
const double EUFS_SIM_RANGE_STD_DEV = 0.0;
const double EUFS_SIM_BEARING_STD_DEV = 0.0;
const double EUFS_SIM_IMU_X_STD_DEV = 0.0;
const double EUFS_SIM_IMU_Y_STD_DEV = 0.0;
const double EUFS_SIM_IMU_HEADING_STD_DEV = 0.0;
const double EUFS_SIM_GPS_X_STD_DEV = 0.0;
const double EUFS_SIM_GPS_Y_STD_DEV = 0.0;

/***** CONTROLS SIM ******/
const double CONTROLS_BEARING_STD_DEV = 0.00045;
const double CONTROLS_RANGE_STD_DEV = 0.03;
const double CONTROLS_IMU_X_STD_DEV = 0.22;
const double CONTROLS_IMU_Y_STD_DEV = 0.22;
const double CONTROLS_IMU_HEADING_STD_DEV = 0.00872665;
const double CONTROLS_GPS_X_STD_DEV = 0.01;
const double CONTROLS_GPS_Y_STD_DEV = 0.01;

const long SEC_TO_NANOSEC = 1e9;
const double IMU_OFFSET = 0.3; //meters; offset from the center of the car
const double LIDAR_OFFSET = 0.3; //meters; offset from the center of the car
const double MAX_CONE_RANGE = 15;
const double TURNING_MAX_CONE_RANGE = 15;
const double VELOCITY_MOVING_TH = 0.1; //meters per second
const double TURNING_TH = 0.2;

const double DIST_FROM_START_LC_TH = 5; //meters; distance from the start for loop closure detection

const double M_DIST_TH = 0.0009;
const double TURNING_M_DIST_TH = 0.0009;


const std::string STEP_INPUT_FILE = "src/isam2/data/step_input.txt";

#define CONE_DATA_TOPIC "/perc_cones"
#define VEHICLE_POS_TOPIC "/filter/pose"
#define VEHICLE_ANGLE_TOPIC "/filter/quaternion"
#define VEHICLE_VEL_TOPIC "/filter/twist"
