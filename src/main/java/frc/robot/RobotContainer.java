// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.TankDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems here:
  private TankDriveSubsystem m_drive = new TankDriveSubsystem(Constants.drive_constants);

  // Controllers here:
  private XboxController m_driverController = new XboxController(Constants.kControllerPort);

  // Robot commands go here:
  // This command runs on autonomous
  private Command m_autoCommand = null;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drive.setDefaultCommand(
      new RunCommand(
        () -> m_drive.arcadeDrive(
          m_driverController.getY(GenericHID.Hand.kLeft),
          m_driverController.getX(GenericHID.Hand.kRight)
          ), m_drive
        )
      );

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Run path following command, then stop at the end.
    // return new TrajectoryCommand(m_drive, 
    //   new Pose2d(0, 0, new Rotation2d(0)), 
    //   List.of(
    //         new Translation2d(1, 1),
    //         new Translation2d(2, -1)
    //     ), 
    //   new Pose2d(3, 0, new Rotation2d(0)));
    return m_autoCommand;
  }
}
