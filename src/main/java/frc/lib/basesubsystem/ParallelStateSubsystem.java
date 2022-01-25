// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.basesubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ODN_State;

public class ParallelStateSubsystem extends SubsystemBase implements ODN_State {
  
  ODN_State[] subsystems;
  public ParallelStateSubsystem(ODN_State... subsystems) {
    this.subsystems = subsystems;
  }

  @Override
  public void setGoalLocation(double pos) {
    for(ODN_State s: subsystems) {
      s.setGoalLocation(pos);
    }
  }

  @Override
  public boolean atGoalLocation() {
    for(ODN_State s: subsystems) {
      if(!s.atGoalLocation()) return false;
    }
    return true;
  }

  @Override
  public void resetPosition() {
    for(ODN_State s: subsystems) {
      s.resetPosition();
    }
  }
}
