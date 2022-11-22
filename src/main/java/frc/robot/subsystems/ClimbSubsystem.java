// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.encoder.ODN_CANCoder;
import frc.lib.motorcontroller.ODN_MotorController;

public class ClimbSubsystem extends SubsystemBase {

  public static class Constants{
    public ODN_MotorController climb;
    public ODN_CANCoder encoder;
    public double lowerLimit;
    public double upperLimit;
  }
  private ODN_MotorController m_climb;
  private ODN_CANCoder m_encoder;
  private double m_lowerLimit;
  private double m_upperLimit;
  
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(Constants constants) {
    m_climb = constants.climb;
    m_encoder = constants.encoder;
    m_lowerLimit = constants.lowerLimit;
    m_upperLimit = constants.upperLimit;
  }
  public void set(double speed) {
    if (m_encoder.getAbsolutePosition() < m_upperLimit && speed > 0) m_climb.set(speed);
    else if (m_encoder.getAbsolutePosition() > m_lowerLimit && speed < 0) m_climb.set(speed);
    else m_climb.set(0);

  }
  @Override
  public void periodic() {
  }
}