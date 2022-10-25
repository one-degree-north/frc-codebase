// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.basesubsystem.ArmSubsystem;

public class TwoLinkArmSubsystem extends SubsystemBase {

  public static class Constants {
    public ArmSubsystem.Constants armlink1, armlink2;
  }
  
  private ArmSubsystem m_armlink1, m_armlink2;

  /** Creates a new HalloweenArmSubsystem. */
  public TwoLinkArmSubsystem(Constants constants) {
    m_armlink1 = new ArmSubsystem(constants.armlink1);
    m_armlink2 = new ArmSubsystem(constants.armlink2);
  }

  public void resetEncoder1() {
    m_armlink1.resetPosition();
  }

  public void resetEncoder2() {
    m_armlink2.resetPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
