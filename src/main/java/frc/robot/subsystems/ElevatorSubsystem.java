// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.encoder.Encoder;
import frc.lib.encoder.ODN_CANCoder;
import frc.lib.motorcontroller.MotorController;
import frc.lib.motorcontroller.ODN_SparkMax;
import frc.lib.motorcontroller.ODN_SparkMax.MotorType;

public class ElevatorSubsystem extends SubsystemBase {

  public static final double EPSILON = 0.01;

  public static class Constants {
    //CAN IDs for spark and encoder
    public int id_spark;
    public int id_enc;

    // Convert encoder output to meters
    public int encoderFactor;

    // Maximum v and a for motion profiling
    public double maxVelocity;
    public double maxAcceleration;

    // PID values
    public double kp, ki, kd;

    // Feedforward values
    public double ks, kg, kv;
  }

  private MotorController m_spark;
  private Encoder m_enc;

  private double m_pos;

  private Constants m_constants;
  
  private ProfiledPIDController m_controller;

  private ElevatorFeedforward m_feedforward;

  public ElevatorSubsystem(Constants constants) {
    this.m_constants = constants;
    m_spark = new ODN_SparkMax(constants.id_spark, MotorType.brushless);
    m_enc = new ODN_CANCoder(constants.id_enc);
    
    m_controller = new ProfiledPIDController(m_constants.kp, m_constants.ki, m_constants.kd, 
        new TrapezoidProfile.Constraints(m_constants.maxVelocity, m_constants.maxAcceleration));
    m_feedforward = new ElevatorFeedforward(m_constants.ks, m_constants.kg, m_constants.kv);
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

    // Add feedforward compensation to speed
    m_spark.setVoltage(speed + m_feedforward.calculate(m_enc.getVelocity()));
  }
}