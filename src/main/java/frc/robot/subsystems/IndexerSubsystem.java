// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.sensor.ODN_ColorSensor;

public class IndexerSubsystem extends SubsystemBase {
  private static final Color RED = new Color(1, 0, 0);
  private static final Color BLUE = new Color(0, 0, 1);

  private static final ColorMatch m_matcher = ODN_ColorSensor.createMatcher(0.8, RED, BLUE);

  public static class Constants {
    public MotorControllerSubsystem motor;
    public ODN_ColorSensor color;
    public DigitalInput breakbeam;
  }
  private MotorControllerSubsystem m_motor;
  private ODN_ColorSensor m_color;
  private DigitalInput m_breakbeam;


  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem(Constants constants) {
    m_motor = constants.motor;
    m_color = constants.color;
    m_breakbeam = constants.breakbeam;
  }

  @Override
  public void periodic() {
    if(m_matcher.matchClosestColor(m_color.getColor()).color.equals(RED)) {
      //TODO: write this code
      // new ShootAwayCommand().schedule();
    }
    if(m_breakbeam.get())
    {
      //shooter stops moving
    }

    // This method will be called once per scheduler run
  }

  public void set(double d) {
    m_motor.set(d);
  }
}
