// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#include <units/length.h>
#include <unser_defined/PID_Coefficients.h>

#define CURAVATURE_DRIVE_MODE 0
#define ARCADEE_DRIVE_MODE 1

#define ROBORIO_LOOP_PERIOD 0.02_s // corresponds to a frequency of 50Hz

// DriveConstants namespace is a location for all constants related to the Drive subsystem
namespace DriveConstants {
  
  // left motor controller IDs
  const int left_talon_id = 1;
  const int left_victor1_id = 2;
  const int left_victor2_id = 3;
  
  // right motor controller IDs
  const int right_talon_id = 0;
  const int right_victor1_id = 4;
  const int right_victor2_id = 5;
  
  // drive PID coefficients
  const PID_Coefficients drive_PID_coefficients(0, 0.1, 2.0e-5, 0);
  const PID_Coefficients gyro_PID_coefficients(0, 0.008333, 0, 0);
  const PID_Coefficients limelight_PID_coefficients(0, 0.035, 2.0e-5, 0);
  
  // control drive mode (either curvature or arcade)
  const int drive_mode = ARCADE_DRIVE_MODE;
  
  const double encoder_filter_cutoff_frequency = 0.1;
  
  const auto track_width = 30_in;
  
  const int gyroscope_cs = 0;
  
}

//IntakeConstants is a location for all constants related to the Intake
