// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.basesubsystem.PneumaticSubsystem;

public class ClimbSubsystem extends SubsystemBase {

  private PneumaticSubsystem m_enable_climber;
  private PneumaticSubsystem m_rotation;
  private MotorControllerSubsystem m_motor;

  public static class Constants {
    public PneumaticSubsystem.Constants enable_climber;
    public PneumaticSubsystem.Constants rotation;
    public MotorControllerSubsystem.Constants motor;
  }

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(Constants constants) {
    m_enable_climber = new PneumaticSubsystem(constants.enable_climber);
    m_rotation = new PneumaticSubsystem(constants.rotation);
    m_motor = new MotorControllerSubsystem(constants.motor);
  }

  public void disable() {
    m_enable_climber.set(Value.kForward);
    m_rotation.set(Value.kForward);
  }

  public void enable() {
    m_enable_climber.set(Value.kReverse);
  }

  public void contractRotation() {
    m_rotation.set(Value.kReverse);
  }

  public void extendRotation() {
    m_rotation.set(Value.kForward);
  }

  public Command raiseTelescopingElevator() {
    //TODO: Write this
    return null;
  }

  public Command dropTelescopingElevator() {
    //TODO: Write this
    return null;
  }
}
