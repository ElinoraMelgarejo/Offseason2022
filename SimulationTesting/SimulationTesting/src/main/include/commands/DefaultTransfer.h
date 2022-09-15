// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <subsystems/Transfer.h>

class DefaultTransfer
    : public frc2::CommandHelper<frc2::CommandBase, DefaultTransfer> {
 public:
  DefaultTransfer(Transfer* m_transfer);

  void Execute() override;

 private:
  Transfer* m_transfer;

};
