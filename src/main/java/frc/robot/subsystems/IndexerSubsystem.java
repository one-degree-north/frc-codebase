// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.sensor.ODN_ColorSensor;
import frc.robot.Constants;
import frc.robot.commands.ShootAwayCommand;

public class IndexerSubsystem extends SubsystemBase {
  private MotorControllerSubsystem m_motor;
  private ODN_ColorSensor m_color;
  private DigitalInput m_input;
  private final Color Red = new Color(1, 0, 0);
  private final Color Blue = new Color(0, 0, 1);
  private ColorMatch m_matcher = ODN_ColorSensor.createMatcher(0.8, Red, Blue);

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem(MotorControllerSubsystem motor, ODN_ColorSensor color, DigitalInput input) {
    m_motor = motor;
    m_color = color;
    m_input = input;
    // m_input.
  }

  @Override
  public void periodic() {
    if(m_matcher.matchClosestColor(m_color.getColor()).color.equals(Red)) {
      //TODO: write this code
      // new ShootAwayCommand().schedule();
    }
    if(m_input.get())
    {
      //shooter stops moving
    }

    // This method will be called once per scheduler run
  }

  public void set(double d) {
    m_motor.set(d);
  }
}
