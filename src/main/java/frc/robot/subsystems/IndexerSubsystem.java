// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.encoder.ODN_Encoder;
import frc.lib.sensor.ODN_Adafruit164Sensor;
import frc.lib.sensor.ODN_ColorSensor;
import frc.robot.RobotContainer;
import frc.robot.commands.IndexerContinueCommand;

public class IndexerSubsystem extends SubsystemBase {

  private static final double DISTANCE_TO_ENABLE = 0;

  private static final Color RED = new Color(1, 0, 0);
  private static final Color BLUE = new Color(0, 0, 1);

  private static final ColorMatch m_matcher = ODN_ColorSensor.createMatcher(0.8, RED, BLUE);

  public static class Constants {
    public MotorControllerSubsystem motor;
    public ODN_ColorSensor color;
    public ODN_Adafruit164Sensor enter_sensor;
    public DigitalInput exit_sensor;
    private ODN_Encoder encoder;
  }
  private MotorControllerSubsystem m_motor;
  private ODN_ColorSensor m_color;
  public ODN_Adafruit164Sensor m_enter_sensor;
  public DigitalInput m_exit_sensor;
  private ODN_Encoder m_encoder;


  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem(Constants constants) {
    m_motor = constants.motor;
    m_color = constants.color;
    m_enter_sensor = constants.enter_sensor;
    m_exit_sensor = constants.exit_sensor;
    m_encoder = constants.encoder;
  }

  public static enum BallColor {
    BLUE, RED, NONE
  }

  public BallColor getColor() {
    Color match_res = m_matcher.matchColor(m_color.getColor()).color;
    if(match_res == null) return BallColor.NONE;
    if(match_res.equals(RED)) return BallColor.RED;
    if(match_res.equals(BLUE)) return BallColor.BLUE;
    return null;
  }

  @Override
  public void periodic() {
    if(m_exit_sensor.get())
    {
      new IndexerContinueCommand(this);
    }
    if(m_enter_sensor.getDistanceInches() < DISTANCE_TO_ENABLE)
    {
      RobotContainer.container.getIndexer().on();
    }

    // This method will be called once per scheduler run
  }

  public void on() {
    m_motor.set(0.8);
  }

  public void off() {
    m_motor.set(0);
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }

  public double getEncoder() {
    return m_encoder.getPosition();
  }
}
