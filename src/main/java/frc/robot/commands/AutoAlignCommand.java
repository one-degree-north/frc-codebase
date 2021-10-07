// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.ODN_Drivebase;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAlignCommand extends CommandBase {
  public static final double EPSILON = 0.5; // in degrees

  private ODN_Drivebase m_drive;
  private LimelightSubsystem m_limelight;
  private Function<Double, Double> m_attenuationFunction;

  public AutoAlignCommand(ODN_Drivebase drive, LimelightSubsystem limelight, Function<Double, Double> attenuationFunction) {
    this.m_drive = drive;
    this.m_limelight = limelight;
    this.m_attenuationFunction = attenuationFunction;
    addRequirements(drive, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Attenuation function returns turning speed from limelight horizontal angle offset
    m_drive.rotate(m_attenuationFunction.apply(m_limelight.getOffsetHorizontal()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_limelight.getOffsetHorizontal())<EPSILON;
  }
}
