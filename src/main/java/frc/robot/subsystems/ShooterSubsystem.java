// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.encoder.ODN_CANCoder;
import frc.lib.motorcontroller.ODN_MotorController;
import frc.robot.subsystems.ClimbSubsystem.Constants;

public class ShooterSubsystem extends SubsystemBase {
  public static class Constants{
    public ODN_MotorController shoot;
    public ODN_MotorController intake;
    public ODN_MotorController index;
  }
  private ODN_MotorController m_shoot;
  private ODN_MotorController m_intake;
  private ODN_MotorController m_index;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(Constants constants) {
    m_shoot = constants.shoot;
    m_intake = constants.intake;
    m_index = constants.index;
  }
  public void setShoot(double speed){
    m_shoot.set(speed);
  }
  public void setIntake(double speed){
    m_intake.set(speed);
  }
  public void setIndex(double speed){
    m_index.set(speed);
  }
  public void disableAll(){
    m_shoot.set(0);
    m_intake.set(0);
    m_index.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
