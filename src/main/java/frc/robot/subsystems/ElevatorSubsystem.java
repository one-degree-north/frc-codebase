// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  public static final double EPSILON = 0.01;

  public static class Constants {
    int id_spark;
    int id_enc;
    // Convert encoder output to meters
    int id_encoderFactor;
  }

  private CANSparkMax m_spark;
  private CANCoder m_enc;

  private double m_pos;

  private Constants m_constants;

  public ElevatorSubsystem(Constants constants) {
    this.m_constants = constants;
    m_spark = new CANSparkMax(constants.id_spark, MotorType.kBrushless);
    m_enc = new CANCoder(constants.id_enc);
  }

  /**
   * Sets a new goal position for the elevator
   * @param pos New goal position in meters
   */
  public void setGoalLocation(double pos) {
    this.m_pos = pos;
  }

  public boolean atGoalLocation() {
    return Math.abs(m_pos-m_enc.getPosition()*m_constants.id_encoderFactor) < EPSILON;
  }

  public void resetPosition() {
    m_enc.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_spark.set((m_pos-m_enc.getPosition()*m_constants.id_encoderFactor));
  }
}
