// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.basesubsystem.SwerveDriveSubsystem;

public class DriveTo extends CommandBase {
  SwerveDriveSubsystem m_drive;
  Pose2d m_finalPoseRelative;
  Pose2d toPose;
  /** Creates a new DriveTo. */
  public DriveTo(SwerveDriveSubsystem drive, Pose2d finalPoseRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_finalPoseRelative = finalPoseRelative;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    toPose = new Pose2d(m_finalPoseRelative.getTranslation().plus(m_drive.getPose().getTranslation()), m_finalPoseRelative.getRotation().plus(m_drive.getPose().getRotation()));
  }

  private static double clamp(double val, double min, double max) {
    if(val < min) val = min;
    if(val > max) val = max;
    return val;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("x");
    m_drive.cartesianDriveRelative(clamp((toPose.getX() - m_drive.getPose().getX()), 0, 0.5), clamp((toPose.getY() - m_drive.getPose().getY()), 0, 0.5), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getPose().getTranslation().getDistance(toPose.getTranslation()) < 0.1;
  }
}
