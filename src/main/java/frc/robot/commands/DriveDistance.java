// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

// Drive the drivebase a certain distance and then stops
public class DriveDistance extends CommandBase {
  /** Creates a new DriveDistance. */
  private TankDrive m_drive;
  private double distance;
  private double startPos;
  public DriveDistance(TankDrive drive, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = drive;
    this.distance = distance;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = m_drive.leftEncoderPosition();
    m_drive.arcadeDrive(1.0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.leftEncoderPosition()-startPos > distance;
  }
}
