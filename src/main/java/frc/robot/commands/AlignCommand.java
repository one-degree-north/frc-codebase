// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.ODN_HolonomicDrivebase;
import frc.robot.subsystems.HoodSubsystem;
import frc.lib.basesubsystem.LimelightSubsystem;

public class AlignCommand extends CommandBase {
  private ODN_HolonomicDrivebase m_drive;
  private LimelightSubsystem m_limelight;
  private Function<Double, Double> m_attenuationFunction;
  private Function<Double, Double> m_hoodFunction; //max angle is 16 degrees. Need to recalculate the angles of the limelight
  private double distance; 

  public AlignCommand(ODN_HolonomicDrivebase drive, LimelightSubsystem limelight,  Function<Double, Double> attenuationFunction, Function<Double, Double> hoodFunction, XboxController joystick) {
    this.m_drive = drive;
    this.m_limelight = limelight;
    this.m_attenuationFunction = attenuationFunction;
    this.m_hoodFunction = hoodFunction;
    addRequirements(drive, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    distance = m_limelight.getOffsetVertical()/27;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.cartesianDriveRelative((m_limelight.getOffsetVertical()/27-distance)/10, 0, m_attenuationFunction.apply(m_limelight.getOffsetHorizontal()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_limelight.getOffsetHorizontal()==0;
  }
}