// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.lib.encoder.Encoder;
import frc.lib.motorcontroller.MotorController;

public class SwerveModule {
  private final MotorController m_driveMotor;
  private final MotorController m_turningMotor;

  private final Encoder m_turningEncoder;

  private final Encoder m_driveEncoder;

  private final SwerveDriveSubsystem.Constants m_constants;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(MotorController driveMotor, MotorController turningMotor, Encoder driveEncoder, Encoder turningEncoder, double angleOff, SwerveDriveSubsystem.Constants constants) {

    m_driveMotor = driveMotor;
    m_turningMotor = turningMotor;

    this.m_turningEncoder = turningEncoder;
    this.m_driveEncoder= driveEncoder;
    m_driveEncoder.setPositionConversionFactor(Math.PI/360*constants.wheelDiameterMeters);
    m_driveEncoder.setVelocityConversionFactor(Math.PI/360*constants.wheelDiameterMeters);
    m_turningEncoder.setPositionConversionFactor(Math.PI/180);
    m_turningEncoder.setVelocityConversionFactor(Math.PI/180);
    resetEncoders(angleOff);

    this.m_constants = constants;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getAngle()));
  }
  double getAng;
  public double getAngle(){
    getAng=m_turningEncoder.getPosition();
    if(getAng>Math.PI){
      getAng-=2*Math.PI;
    }
    else if(getAng<-Math.PI){
      getAng+=2*Math.PI;
    }
    return getAng;
  }
  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    double angle = state.angle.getRadians();
    int flipper = 1;
    if(Math.abs(angle-getAngle())<=2*Math.PI/3) {

    } else if(angle-getAngle()>0) {
      flipper = -flipper;
      angle -= Math.PI;
    } else if(angle-getAngle()<0) {
      flipper = -flipper;
      angle += Math.PI;
    }
    m_driveMotor.set(state.speedMetersPerSecond/m_constants.maxSpeedMetersPerSecond*flipper);
    m_turningMotor.set((angle-getAngle())/(2*Math.PI));
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders(double angOff) {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition((m_turningEncoder.getAbsolutePosition()%360-angOff)/180 * Math.PI);
  }
 
}