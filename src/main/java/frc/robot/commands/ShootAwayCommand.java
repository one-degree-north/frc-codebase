// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.gyro.ODN_Gyro;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.MotorControllerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ShootAwayCommand extends CommandBase {
  /** Creates a new ShootAwayCommand. */
  private IndexerSubsystem m_indexer;
  private SwerveDriveSubsystem m_swerve;
  private MotorControllerSubsystem m_shooter;
  private ODN_Gyro m_gyro;
  public ShootAwayCommand(IndexerSubsystem indexer, SwerveDriveSubsystem swerve, MotorControllerSubsystem shooter, ODN_Gyro gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_indexer = indexer;
    m_swerve = swerve;
    m_shooter = shooter;
    m_gyro = gyro;
    addRequirements(m_indexer, m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.cartesianDriveAbsolute(0, 0, m_gyro.getYaw().getRadians()-Math.PI/2);
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
