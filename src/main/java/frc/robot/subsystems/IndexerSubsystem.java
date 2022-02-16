// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.encoder.ODN_Encoder;
import frc.lib.sensor.ODN_Adafruit164Sensor;
import frc.lib.sensor.ODN_ColorSensor;

public class IndexerSubsystem extends SubsystemBase {

  private static final double DISTANCE_TO_ENABLE = 0;
  private static final Color RED = new Color(0.575927734375, 0.3154296875, 0.10888671875);
  private static final Color BLUE = new Color(0.153076171875, 0.386962890625, 0.460205078125);

  private static final ColorMatch m_matcher = ODN_ColorSensor.createMatcher(0.95, RED, BLUE);

  public static class Constants {
    public MotorControllerSubsystem.Constants indexer;
    public MotorControllerSubsystem.Constants feeder;
    public ODN_ColorSensor color;
    public ODN_Adafruit164Sensor enter_sensor;
    public DigitalInput exit_sensor;
    public ODN_Encoder encoder;
  }
  private MotorControllerSubsystem m_indexer;
  private MotorControllerSubsystem m_feeder;
  private ODN_ColorSensor m_color;
  private ODN_Adafruit164Sensor m_enter_sensor;
  private DigitalInput m_exit_sensor;
  private ODN_Encoder m_encoder;


  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem(Constants constants) {
    m_indexer = new MotorControllerSubsystem(constants.indexer);
    m_feeder = new MotorControllerSubsystem(constants.feeder);
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
  }

  public void onboth() {
    m_feeder.set(0.8);
    m_indexer.set(0.8);
  }
  public void onfeeder() {
    m_feeder.set(0.8);
    m_indexer.set(0);
  }

  public void off() {
    m_feeder.set(0);
    m_indexer.set(0);
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }

  public double getEncoder() {
    return m_encoder.getPosition();
  }

  public Trigger ballAtEntrance() {
    return new Trigger(()->m_enter_sensor.getDistanceInches() < DISTANCE_TO_ENABLE);
  }

  public Trigger ballAtExit() {
    return new Trigger(()->getExitSensor());
  }

  public boolean getExitSensor() {
    return m_exit_sensor.get();
  }
}