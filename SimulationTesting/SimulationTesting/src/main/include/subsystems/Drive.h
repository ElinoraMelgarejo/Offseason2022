// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// all frc/frc2 includes
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/filter/LinearFilter.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "ctre/Phoenix.h"
#include "Constants.h"

// The Drive subsystem will represent an abstraction of the drivebase to allow for general robot mobility
class Drive : public frc2::SubsystemBase {
  
  public:
    Drive();
    void Periodic() override;
    // Curvature Drive
    void CurvatureDrive(double forward, double rotate);
    // Arcade Drive
    void ArcadeDrive(double forward, double rotate);
    // Encoder Reset
    void ResertEncoder();
    double GetVelocity();
    double GetPosition();
    void ResetAngle();
    double GetAngle();
    double GetUnfilteredVelocity();
  
    // Converting the linear &angular velocity of the chassis into left & right velocity
    frc::DifferentialDriveWheelSpeeds GetWheelSpeeds(frc::ChassisSpeeds chs_spd);
    void DriveToDistance(double setpoint);
  
  private:
  
    // Left Motor Controllers
    WPI_TalonSRX left_talon = {DriveConstants::left_talon_id);
    WPI_VictorSPX left_victor1 = {DriveConstants::left_victor1_id};
    WPI_VictorSPX left_victor2 = {DriveConstants::left_victor2_id};                           
    
    // Right Motor Controllers
    WPI_TalonSRX right_talon = {DriveConstants::right_talon_id};
    WPI_VictorSPX right_victor1 = {DriveConstants::right_victor1_id};
    WPI_VictorSPX right_victor2 = {DriveConstants::right_victor2_id};
                               
    frc::MotorControllerGroup left{left_talon, left_victor1, left_victor2};
    frc::MotorControllerGroup right{right_talon, right_victor1, right_victor2};
                               
    frc::DifferentialDrive drive{left, right};
    frc::DifferentialDriveKinematics drive_kinematics{DriveConstants::track_width};
    frc::LinearFilter<double> encoder_filter = frc::LinearFilter<double>::SinglePoleIIR(DriveConstants::encoder_filter_cutoff_frequency, ROBORIO_LOOP_PERIOD);
    
    frc::ADXRS450_Gyro gyroscope;
                               
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("drive");
                               
                               
};                           
