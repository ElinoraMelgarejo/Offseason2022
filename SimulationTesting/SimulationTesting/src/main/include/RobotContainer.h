// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


// all frc/frc2 includes
#include <frc/XboxController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Button.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/ParallelRaceGroup.h>

// all subsystem includes
#include <subsystems/Drive.h>
#include <subsystems/Intake.h>
#include <subsystems/Transfer.h>
#include <subsystems/Limelight.h>
#include <subsystems/Launcher.h>
#include <subsystems/Climber.h>

// all command includes
#include <commands/DefaultDrive.h>
#include <commands/DefaultTransfer.h>
#include <commands/DefaultIntake.h>
#include <commands/DefaultLimelight.h>
#include <commands/DefaultLauncher.h>
#include <commands/TurnToAngleGyro.h>
#include <commands/DefaultClimber.h>
#include <commands/LaunchAtSpeed.h>
#include <commands/DriveToDistance.h>
#include <commands/TurnToAngleLimelight.h>

// all other includes
#include <Constants.h>


/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {

 public:
  // Constructor definition
  // No arguments necessary
  RobotContainer();
  
  // GetAutonomousCommand should return a pointer to an frc2::Command object
  frc2::Command* GetAutonomousCommand();

 private:
 
  // Drive subsystem
  Drive m_drive;
  
  // Intake subsystem
  Intake m_intake;
  
  // Transfer subsystem
  Transfer m_transfer;
  
  // Limelight subsystem
  Limelight m_limelight;
  
  // Launcher subsystem
  Launcher m_launcher;
  
  // Climber subsystem
  Climber m_climber;
  

  void ConfigureButtonBindings();
};
