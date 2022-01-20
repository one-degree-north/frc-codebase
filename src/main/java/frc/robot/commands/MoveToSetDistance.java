// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.basesubsystem.LimelightSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;

public class MoveToSetDistance extends CommandBase {
  private double endDistance;
  private SwerveDriveSubsystem m_swerve;
  private LimelightSubsystem m_limelight;
  private XboxController m_joystick;
  private static Function<Double, Double> m_attenuationFunction = LimelightSubsystem.linearAttenuation(27);
  private final double d = 10;
  private final double h = 11;
  private final double theta = Math.PI/6;
  /** Creates a new MoveToSetDistance. */
  public MoveToSetDistance(SwerveDriveSubsystem m_swerve, LimelightSubsystem m_limelight, XboxController m_joystick, double endDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.endDistance = endDistance;
    addRequirements(m_swerve, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = (h-d)/Math.tan(theta+m_limelight.getOffsetVertical());
    double sidespeed = -m_joystick.getLeftX();
    m_swerve.cartesianDriveRelative(distance-endDistance, sidespeed, m_attenuationFunction.apply(m_limelight.getOffsetHorizontal()));
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
