// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems here:
  private SwerveDriveSubsystem m_drive = new SwerveDriveSubsystem(Constants.swerveConstants);
  private LimelightSubsystem m_limelight = new LimelightSubsystem();

  // Controllers here:
  private XboxController m_controller = new XboxController(0);

  // Robot commands go here:
  // This command runs on autonomous
  private Command m_autoCommand = null;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new RunCommand(() -> 
      m_drive.drive(-modifyAxis(m_controller.getY(Hand.kLeft)), 
      -modifyAxis(m_controller.getX(Hand.kLeft)),
      modifyAxis(m_controller.getX(Hand.kRight))), 
      m_drive));
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton button = new JoystickButton(m_controller, XboxController.Button.kA.value);
    button.whenPressed(new AutoAlignCommand(m_drive, m_limelight, LimelightSubsystem.linearAttenuation(27)));
    JoystickButton button2 = new JoystickButton(m_controller, XboxController.Button.kB.value);
    button2.whenPressed(new InstantCommand(()->m_drive.zeroGyroscope(), m_drive));
  }
  
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // m_drive.resetAllEncoders();

  //   SwerveTrajectoryCommand traj1 = new SwerveTrajectoryCommand(m_drive,
  //   List.of(
  //     new Translation2d(3, 0)), 
  //     new Pose2d(2,0.75, Rotation2d.fromDegrees(-90)));


  // SwerveTrajectoryCommand traj2 = new SwerveTrajectoryCommand(m_drive,
  //   List.of(), 
  //     new Pose2d(1.5,0, new Rotation2d(0))); 

  //   return traj1.andThen(traj2);

    // SwerveTrajectoryCommand traj1 = new SwerveTrajectoryCommand(m_drive,
    // List.of(
    //   new Translation2d(0.2,-0.5),
    //   new Translation2d(0.75, -0.75),
    //   new Translation2d(1.25,-0.5),
    //   new Translation2d(1.5,0),
    //   new Translation2d(1.25, 0.5),
    //   new Translation2d(0.75, 0.75),
    //   new Translation2d(0.2, 0.5)),

    //   new Pose2d(0,0, Rotation2d.fromDegrees(180)));


    // SwerveTrajectoryCommand traj2 = new SwerveTrajectoryCommand(m_drive,
    // List.of(
    //   new Translation2d(0.2,0.5),
    //   new Translation2d(0.75, 0.75),
    //   new Translation2d(1.25,0.5),
    //   new Translation2d(1.5,0),
    //   new Translation2d(1.25,-0.5),
    //   new Translation2d(0.75,-0.75),
    //   new Translation2d(0.2,-0.5)),
    //   new Pose2d(0,0, Rotation2d.fromDegrees(0)));

    // SwerveTrajectoryCommand test = new SwerveTrajectoryCommand(m_drive, List.of(new Translation2d(-1.5,0)), 
    //   new Pose2d(-2.75, 0, Rotation2d.fromDegrees(90)));

    return m_autoCommand;

  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
