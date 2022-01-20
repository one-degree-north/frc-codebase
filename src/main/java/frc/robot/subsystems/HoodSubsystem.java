// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.encoder.ODN_CANCoder;
import frc.lib.motorcontroller.ODN_MotorController;

public class HoodSubsystem extends SubsystemBase {

  public static class Constants {
    public ODN_MotorController motor;
    public ODN_CANCoder encoder;
    public int minValue;// change
    public int maxValue; //change
    public double conversion;
    public double kp, ki, kd;
    public double maxVelocity, maxAcceleration;
  }
  private ODN_MotorController m_motor;
  private ODN_CANCoder m_encoder;
  private int minValue;// change
  private int maxValue; //change
  private double m_pos = 55;
  private ProfiledPIDController pid;
  private double EPSILON = 0.5;


  /** Creates a new Hood. */
  public HoodSubsystem(Constants constants) {
    m_motor = constants.motor;
    m_encoder = constants.encoder;
    m_encoder.setPositionConversionFactor(constants.conversion);
    m_encoder.setVelocityConversionFactor(constants.conversion);
    m_encoder.setPosition(55);
    minValue = constants.minValue;
    maxValue = constants.maxValue;
    pid =  new ProfiledPIDController(constants.kp, constants.ki, constants.kd, 
        new TrapezoidProfile.Constraints(constants.maxVelocity, constants.maxAcceleration));


  }

  public void set(double angle){
      if(angle<=55){
        m_pos = 55;
      }
      else if(angle>=70){
        m_pos = 70;
      }
      else{
        m_pos = angle;
      }
      

  }

  public double getHood(){
    return m_encoder.getPosition();
  }

  public boolean atGoalLocation(){
    return Math.abs(m_pos - m_encoder.getPosition())<EPSILON;
  }
  
  public void resetPosition(){
    m_encoder.setPosition(55);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double speed = pid.calculate(m_encoder.getPosition(), m_pos);
    if(Math.abs(speed)<EPSILON){
      speed = 0;
    }
    if(m_encoder.getPosition()>=55 && m_encoder.getPosition()<=70){
      
      m_motor.setVoltage(speed);
    }

  }
  


}
