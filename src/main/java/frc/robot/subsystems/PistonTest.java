// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.basesubsystem.PneumaticSubsystem;


public class PistonTest extends SubsystemBase {
  public static class Constants {
    public PneumaticSubsystem.Constants piston;
  }
  
  private PneumaticSubsystem m_piston;

  /** Creates a new PistonTest. */
  public PistonTest(Constants constants) {
    m_piston = new PneumaticSubsystem(constants.piston);
  }

  public void retract() {
    m_piston.set(Value.kForward);
  }
  
  public void extend() {
    m_piston.set(Value.kReverse);
  }

  public void toggle() {
    m_piston.toggle();
  }

  public void off() {
    m_piston.set(Value.kOff);
  }

  @Override
  public void periodic() {}
}
