// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.ODN_Drivebase;
import frc.lib.ODN_HolonomicDrivebase;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAlignCommand extends CommandBase {
  public static final double EPSILON = 0.5; // in degrees

  private ODN_HolonomicDrivebase m_drive;
  private LimelightSubsystem m_limelight;
  private Function<Double, Double> m_attenuationFunction;
  private XboxController m_joystick;

  public AutoAlignCommand(ODN_HolonomicDrivebase drive, LimelightSubsystem limelight, Function<Double, Double> attenuationFunction, XboxController joystick) {
    this.m_drive = drive;
    this.m_limelight = limelight;
    this.m_attenuationFunction = attenuationFunction;
    addRequirements(drive, limelight);
    this.m_joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Attenuation function returns turning speed from limelight horizontal angle offset
    double sidespeed = -m_joystick.getX(Hand.kLeft);
    m_drive.cartesianDriveRelative(0, sidespeed, m_attenuationFunction.apply(m_limelight.getOffsetHorizontal()));
    //m_drive.polarDrive(sidespeed, 90-m_drive.getYaw()-m_limelight.getOffsetHorizontal(), m_attenuationFunction.apply(m_limelight.getOffsetHorizontal()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
