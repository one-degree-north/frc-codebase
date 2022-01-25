// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.basesubsystem;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ODN_State;
import frc.lib.encoder.ODN_Encoder;
import frc.lib.motorcontroller.ODN_MotorController;

public class ElevatorSubsystem extends SubsystemBase implements ODN_State {

  public static final double EPSILON = 0.01;

  public static class Constants {
    //CAN IDs for spark and encoder
    public ODN_MotorController motor;
    public ODN_Encoder encoder;

    // Convert encoder output to meters
    public double encoderFactor;

    // Maximum v and a for motion profiling
    public double maxVelocity;
    public double maxAcceleration;

    // PID values
    public double kp, ki, kd;

    // Feedforward values
    public double ks, kg, kv;
  }

  private ODN_MotorController m_motor;
  private ODN_Encoder m_encoder;

  private double m_pos;

  public Constants m_constants;
  
  private ProfiledPIDController m_controller;

  private ElevatorFeedforward m_feedforward;

  public ElevatorSubsystem(Constants constants) {
    this.m_constants = constants;
    m_motor = constants.motor;
    m_encoder = constants.encoder;
    m_encoder.setPositionConversionFactor(constants.encoderFactor);
    m_encoder.setVelocityConversionFactor(constants.encoderFactor);
    
    m_controller = new ProfiledPIDController(m_constants.kp, m_constants.ki, m_constants.kd, 
        new TrapezoidProfile.Constraints(m_constants.maxVelocity, m_constants.maxAcceleration));
    m_feedforward = new ElevatorFeedforward(m_constants.ks, m_constants.kg, m_constants.kv);
  }

  @Override
  public void setGoalLocation(double pos) {
    this.m_pos = pos;
  }

  @Override
  public boolean atGoalLocation() {
    return Math.abs(m_pos-m_encoder.getPosition()) < EPSILON;
  }

  @Override
  public void resetPosition() {
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // Calculate speed with PID and motion profiling
    double speed = m_controller.calculate(m_encoder.getPosition(), m_pos);

    // Add feedforward compensation to speed
    m_motor.setVoltage(speed + m_feedforward.calculate(m_encoder.getVelocity()));
  }
}