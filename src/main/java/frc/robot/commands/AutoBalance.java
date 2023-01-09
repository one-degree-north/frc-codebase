// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.basesubsystem.SwerveDriveSubsystem;
import frc.lib.gyro.ODN_AHRS;

public class AutoBalance extends CommandBase {
  ODN_AHRS m_navx;
  SwerveDriveSubsystem m_drive;
  Rotation2d m_initpitch;
  double m_deadband;
  double m_speed;
  boolean balanceMode = false;

  /** Creates a new AutoBalance. */
  public AutoBalance(ODN_AHRS navx, SwerveDriveSubsystem drive, Rotation2d initpitch, double deadband, double speed) {
    m_navx = navx;
    m_drive = drive;
    m_initpitch = initpitch;
    m_deadband = deadband;
    m_speed = speed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balanceMode = false;
    m_drive.cartesianDriveRelative(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_navx.getPitch().getDegrees() >= m_initpitch.getDegrees() + m_deadband || 
    m_navx.getPitch().getDegrees() <= m_initpitch.getDegrees() - m_deadband) balanceMode = true;

    else balanceMode = false;

    if (balanceMode) {
      if (m_navx.getPitch().getDegrees() >= m_initpitch.getDegrees() + m_deadband)
        m_drive.cartesianDriveRelative(-m_speed, 0, 0);
      if (m_navx.getPitch().getDegrees() <= m_initpitch.getDegrees() - m_deadband)
        m_drive.cartesianDriveRelative(m_speed, 0, 0);
    }

    if (!balanceMode) {
      m_drive.cartesianDriveAbsolute(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    balanceMode = false;
    m_drive.cartesianDriveAbsolute(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
