// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.basesubsystem.LimelightSubsystem;
import frc.lib.basesubsystem.OakSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;

public class MoveToSetDistance extends CommandBase {
  private double endDistance;
  private SwerveDriveSubsystem m_swerve;
  private OakSubsystem m_oak;
  private XboxController m_joystick;
  private Function<Double, Double> m_func = LimelightSubsystem.linearAttenuation(70);
  /** Creates a new MoveToSetDistance. */
  public MoveToSetDistance(SwerveDriveSubsystem swerve, OakSubsystem oak, XboxController joystick, double endDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.endDistance = endDistance;
    m_swerve = swerve;
    m_oak = oak;
    m_joystick = joystick;
    addRequirements(swerve, oak);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = m_oak.getDetections()[0].z;
    double sidespeed = -m_joystick.getLeftX();
    double forwardspeed = m_joystick.getLeftY();
    endDistance -= forwardspeed/100;
    m_swerve.cartesianDriveRelative(distance-endDistance, sidespeed, m_func.apply(Math.atan2(-m_oak.getDetections()[0].x, m_oak.getDetections()[0].z)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
