//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile double q[4];	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickQuaternionUpdate(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz, double deltat);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
