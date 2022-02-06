// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerRunCommand extends CommandBase {
  private IndexerSubsystem m_indexer;

  public IndexerRunCommand(IndexerSubsystem indexer) {
    this.m_indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    m_indexer.onboth();
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.off();
  }
  
  @Override
  public boolean isFinished() {
    return m_indexer.getExitSensor();
  }
}
