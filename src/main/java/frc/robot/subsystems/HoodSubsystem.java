// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.encoder.ODN_CANCoder;
import frc.lib.motorcontroller.ODN_MotorController;

public class HoodSubsystem extends SubsystemBase {

  public static class Constants {
    public ODN_MotorController motor;
    private ODN_CANCoder encoder;
  }
  private ODN_MotorController m_motor;
  private ODN_CANCoder m_encoder;

  /** Creates a new Hood. */
  public HoodSubsystem(Constants constants) {
    m_motor = constants.motor;
    m_encoder = constants.encoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
