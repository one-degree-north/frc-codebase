// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ElevatorHeightCommand extends CommandBase {

  private static final double EPSILON = 10;

  private double m_goal;
  private ClimbSubsystem m_climber;
  /** Creates a new ElevatorHeightCommand. */
  public ElevatorHeightCommand(ClimbSubsystem climber, double goal) {
    m_climber = climber;
    m_goal = goal;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_climber.setMotor(m_goal-m_climber.getMotorLocation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;//return Math.abs(m_goal-m_climber.getMotorLocation()) < EPSILON;
  }
}
