// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.ODN_State;

/**
 * ArmCommand command is used for moving an arm-elevator combination
 * in a horizontal line under the pivot of the arm
 * The elevator moves the arm up and down
 * The arm and elevator move in order to keep the end of the arm from moving up and down
 * as it moves horizontally
 */
public class ArmCommand extends CommandBase {
  private ODN_State m_arm;
  private ODN_State m_ele;
  private TrapezoidProfile trapezoidProfile;
  
  private double startHeight;
  private double armLength;

  /** Creates a new ArmCommand. */
  public ArmCommand(ODN_State arm, ODN_State ele, double start_position, double end_position, double startHeight, double armLength) {
    this.m_arm = arm;
    this.m_ele = ele;
    trapezoidProfile =  new TrapezoidProfile(
      new TrapezoidProfile.Constraints(0.05, 1000), 
      new TrapezoidProfile.State(end_position, 0), 
      new TrapezoidProfile.State(start_position, 0)
    );

    this.startHeight = startHeight;
    this.armLength = armLength;
    
    addRequirements(m_arm, m_ele);
  }

  double startTime;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis()/1000.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = trapezoidProfile.calculate(System.currentTimeMillis()/1000.0-startTime).position;
    double theta = -Math.acos(x/armLength);
    double h = Math.sin(theta) * armLength;
    m_ele.setGoalLocation(h+startHeight);
    m_arm.setGoalLocation(theta);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trapezoidProfile.isFinished(System.currentTimeMillis()/1000.0-startTime);
  }
}
