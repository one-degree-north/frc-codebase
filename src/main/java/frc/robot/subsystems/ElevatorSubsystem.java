// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  public static final double EPSILON = 0.01;

  public static class Constants {
    //CAN IDs for spark and encoder
    int id_spark;
    int id_enc;

    // Convert encoder output to meters
    int encoderFactor;

    // Motor speed required to counter gravity force
    double gravityCompensation;

    // Maximum v and a for motion profiling
    double maxVelocity;
    double maxAcceleration;

    // PID values
    double p, i, d;
  }

  private CANSparkMax m_spark;
  private CANCoder m_enc;

  private double m_pos;

  private Constants m_constants;
  
  private ProfiledPIDController m_controller;

  public ElevatorSubsystem(Constants constants) {
    this.m_constants = constants;
    m_spark = new CANSparkMax(constants.id_spark, MotorType.kBrushless);
    m_enc = new CANCoder(constants.id_enc);
    
    m_controller = new ProfiledPIDController(m_constants.p, m_constants.i, m_constants.d, 
        new TrapezoidProfile.Constraints(m_constants.maxVelocity, m_constants.maxAcceleration));
  }

  /**
   * Sets a new goal position for the elevator
   * @param pos New goal position in meters
   */
  public void setGoalLocation(double pos) {
    this.m_pos = pos;
  }

  /**
   * Checks whether the elevator has reached the goal location
   * @return true if elevator has reached goal location
   */
  public boolean atGoalLocation() {
    return Math.abs(m_pos-m_enc.getPosition()*m_constants.encoderFactor) < EPSILON;
  }

  public void resetPosition() {
    m_enc.setPosition(0);
  }

  @Override
  public void periodic() {
    // Calculate speed with PID and motion profiling
    double speed = m_controller.calculate(m_enc.getPosition()*m_constants.encoderFactor, m_pos);

    // Add gravity compensation to speed
    m_spark.set(speed + m_constants.gravityCompensation);
  }
}
