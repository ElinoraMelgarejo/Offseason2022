// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <subsystems/Climber.h>

class DefaultClimber
    : public frc2::CommandHelper<frc2::CommandBase, DefaultClimber> {
 public:
  DefaultClimber(Climber* m_climber);

  void Execute() override;

  private:
    Climber* m_climber;

};
