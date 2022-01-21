// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.basesubsystem.MotorControllerSubsystem;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public static class Constants {
    public MotorControllerSubsystem.Constants motor;
    public double shoot_speed;
  }
  
  private MotorControllerSubsystem m_motors;
  private double shoot_speed;

  public ShooterSubsystem(Constants constants) {
    m_motors = new MotorControllerSubsystem(constants.motor);
    shoot_speed = constants.shoot_speed;
  }

  public void on() {
    m_motors.setSpeed(shoot_speed);
  }

  public void off() {
    m_motors.setSpeed(0);
  }
}
