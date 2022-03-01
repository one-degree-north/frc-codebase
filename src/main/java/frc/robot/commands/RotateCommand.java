// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.basesubsystem.SwerveDriveSubsystem;

public class RotateCommand extends CommandBase {
  private SwerveDriveSubsystem m_swerve;

  /** Creates a new DriveBackCommand. */
  public RotateCommand(SwerveDriveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    addRequirements(m_swerve);
  }

  Rotation2d start;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = m_swerve.getPose().getRotation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.rotate(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_swerve.getPose().getRotation().minus(start)).getRadians() < Math.PI + 0.1
    && (m_swerve.getPose().getRotation().minus(start)).getRadians() > Math.PI-0.1;
  }
}
