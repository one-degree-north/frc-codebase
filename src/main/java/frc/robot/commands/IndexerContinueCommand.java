package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerContinueCommand extends CommandBase {
    private static final double DISTANCE_TO_TRAVEL = 50;
  
    private IndexerSubsystem m_indexer;
  
    public IndexerContinueCommand(IndexerSubsystem indexer) {
      this.m_indexer = indexer;
      addRequirements(indexer);
    }
  
    @Override
    public void initialize() {
      m_indexer.resetEncoder();
      m_indexer.onshoot();
    }
  
    @Override
    public void end(boolean interrupted) {
      m_indexer.off();
    }
    
    @Override
    public boolean isFinished() {
      return m_indexer.getEncoder() > DISTANCE_TO_TRAVEL;
    }
}
