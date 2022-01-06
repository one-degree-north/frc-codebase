// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motorcontroller.ODN_MotorController;

public class MotorControllerSubsystem extends SubsystemBase {
  public static class Constants {
    public ODN_MotorController group;
  }
  private ODN_MotorController m_controller;

  /** Creates a new ODN_MotorControllerSubsystem. */
  public MotorControllerSubsystem(Constants constants) {
    m_controller = constants.group;
  }

  public void setSpeed(double speed){
    m_controller.set(speed);
  }
}
