// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <subsystems/Limelight.h>

class DefaultLimelight
    : public frc2::CommandHelper<frc2::CommandBase, DefaultLimelight> {
 public:
  DefaultLimelight(Limelight* m_limelight);

  void Execute() override;

 private:
  Limelight* m_limelight;

};
