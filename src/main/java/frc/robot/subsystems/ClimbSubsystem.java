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
    public MotorControllerSubsystem.Constants motor;
  }

  public static final double TOP = 0;
  public static final double BOTTOM = 0;
  public static final double TRANSFER = 0;

  private PneumaticSubsystem m_enable_climber;
  private PneumaticSubsystem m_rotation;
  private MotorControllerSubsystem m_motor;
  private boolean enabled = false;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(Constants constants) {
    m_enable_climber = new PneumaticSubsystem(constants.enable_climber);
    m_rotation = new PneumaticSubsystem(constants.rotation);
    m_motor = new MotorControllerSubsystem(constants.motor);
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
    if(enabled)
      enable();
    else
      disable();
  }

  public void contractRotation() {
    m_rotation.set(Value.kReverse);
  }

  public void extendRotation() {
    m_rotation.set(Value.kForward);
  }

  public void setMotor(double speed) {
    m_motor.set(speed);
  }

  public double getMotorLocation() {
    return m_motor.getPosition();
  }
}
