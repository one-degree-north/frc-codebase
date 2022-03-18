// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.basesubsystem.PneumaticSubsystem;

public class ClimbSubsystem extends SubsystemBase {

  public static class Constants {
    public PneumaticSubsystem.Constants enable_climber;
    public PneumaticSubsystem.Constants rotation;
    public MotorControllerSubsystem.Constants motorLeft;
    public MotorControllerSubsystem.Constants motorRight;
  }

  public static final double TOP = 0;
  public static final double BOTTOM = 0;
  public static final double TRANSFER = 0;

  private PneumaticSubsystem m_enable_climber;
  private PneumaticSubsystem m_rotation;
  private MotorControllerSubsystem m_motorLeft;
  private MotorControllerSubsystem m_motorRight;
  private boolean enabled = false;
  private boolean extended = false;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(Constants constants) {
    m_enable_climber = new PneumaticSubsystem(constants.enable_climber);
    m_rotation = new PneumaticSubsystem(constants.rotation);
    m_motorLeft = new MotorControllerSubsystem(constants.motorLeft);
    m_motorRight = new MotorControllerSubsystem(constants.motorRight);
  }

  public void disable() {
    m_enable_climber.set(Value.kForward);
    m_rotation.set(Value.kForward);
    enabled = false;
  }

  public void enable() {
    m_enable_climber.set(Value.kReverse);
    enabled = true;
  }

  public void toggle() {
    enabled = !enabled;
    if (enabled)
      enable();
    else
      disable();
  }

  public void contractRotation() {
    m_rotation.set(Value.kReverse);
    extended = false;
  }

  public void extendRotation() {
    m_rotation.set(Value.kForward);
    extended = true;
  }

  public void setMotor(double speed) {
    
    /*
    -92000 is the value for the lower limit of the climber. Do not change the lower limit.
    -3000 is for the upper limit. Try changing this around to be slightly bigger to get the
    arms to extend higher. -2000 would probably be a good place to start and keep going up from
    there if you need to, since ideally we don't want to overextend.
    */

    if(speed < 0) { //lower limit -- DO NOT CHANGE
      if(getLeftMotorLocation() > -92000) {
        m_motorLeft.set(speed);
      } else {
        m_motorLeft.set(0);
      }
      if(getRightMotorLocation() > -92000) {
        m_motorRight.set(speed);
      } else {
        m_motorRight.set(0);
      }
    }
    else if(speed > 0){ //upper limit -- number to be changed
      if(getLeftMotorLocation() < -3000) {
        m_motorLeft.set(speed);
      } else {
        m_motorLeft.set(0);
      }
      if(getRightMotorLocation() < -3000) {
        m_motorRight.set(speed);
      } else {
        m_motorRight.set(0);
      }
    } else {
      m_motorLeft.set(0);
      m_motorRight.set(0);
    }
    
  }

  public double getLeftMotorLocation() {
    return m_motorLeft.getPosition();
  }
  public double getRightMotorLocation() {
    return m_motorRight.getPosition();
  }

  public void toggleRotation() {
    extended = !extended;
    if (extended)
      extendRotation();
    else
      contractRotation();
  }
}
