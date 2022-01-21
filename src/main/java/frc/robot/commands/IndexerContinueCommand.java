// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerContinueCommand extends CommandBase {

  private static final double DISTANCE_TO_TRAVEL = 0;

  IndexerSubsystem m_indexer;
  /** Creates a new IndexerContinueCommand. */
  public IndexerContinueCommand(IndexerSubsystem indexer) {
    this.m_indexer = indexer;
    addRequirements(indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexer.on();
    m_indexer.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_indexer.getEncoder() > DISTANCE_TO_TRAVEL;
  }
}
