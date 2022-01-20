// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.basesubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
  public static class Constants {
    public DoubleSolenoid[] solenoids;
  }
  private DoubleSolenoid[] m_solenoids;

  /** Creates a new PneumaticSubsystem. */
  public PneumaticSubsystem(Constants constants) {
    m_solenoids = constants.solenoids;
    for(DoubleSolenoid s: m_solenoids) {
      s.set(Value.kReverse);
    }
  }

  public void toggle() {
    for(DoubleSolenoid s: m_solenoids) {
      s.toggle();
    }
  }

  public void set(Value direction) {
    for(DoubleSolenoid s: m_solenoids) {
      s.set(direction);
    }
  }
}
