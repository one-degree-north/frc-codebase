// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems here:
  // private SwerveDriveSubsystem m_drive = new SwerveDriveSubsystem(Constants.swerveDriveSubsystemConstants);
  // private LimelightSubsystem m_limelight = new LimelightSubsystem();
  // private IntakeSubsystem m_intake = new IntakeSubsystem(Constants.intakeSubsystemConstants);
  // private IntakeSubsystem m_gearIntake = new IntakeSubsystem(Constants.gearIntakeSybsystemConstants);
  private Compressor m_compressor = new Compressor(17);
  // Controllers here:
  private XboxController m_controller = new XboxController(0);

  // Robot commands go here:
  // This command runs on autonomous
  private Command m_autoCommand = null;

  //chungus
  public static boolean isReversed = false;
  public static boolean isLocked = false;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // m_drive.setDefaultCommand(new RunCommand(() -> 
    //   m_drive.drive(-m_controller.getY(Hand.kLeft), 
    //   -m_controller.getX(Hand.kLeft),
    //   -m_controller.getX(Hand.kRight),false, isReversed, isLocked), 
    //   m_drive));
    
    m_compressor.setClosedLoopControl(true);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //JoystickButton button = new JoystickButton(m_controller, XboxController.Button.kA.value);
    //button.toggleWhenPressed(new AutoAlignCommand(m_drive, m_limelight, LimelightSubsystem.linearAttenuation(27)));

    // JoystickButton button = new JoystickButton(m_controller, XboxController.Button.kB.value);
    // button.whenPressed(new SwerveTrajectoryCommand(m_drive, List.of(new Translation2d(-1.5,0)), 
    // new Pose2d(-2.75, 0, Rotation2d.fromDegrees(0))));

    JoystickButton intakeBtn = new JoystickButton(m_controller, XboxController.Button.kBumperLeft.value);
    // intakeBtn.whenPressed(() ->
    //   m_intake.setSpeed(0.5));
    // intakeBtn.whenReleased(() ->
    //   m_intake.setSpeed(0));

    //   JoystickButton gearIntakeBtn = new JoystickButton(m_controller, XboxController.Button.kBumperRight.value);
    // gearIntakeBtn.whenPressed(() ->
    //   m_gearIntake.setSpeed(0.25));
    // gearIntakeBtn.whenReleased(() ->
    //   m_gearIntake.setSpeed(0));

      JoystickButton reverseBtn = new JoystickButton(m_controller, XboxController.Button.kA.value);
    reverseBtn.whenPressed(() -> 
      isReversed = !isReversed);

      JoystickButton lockBtn1 = new JoystickButton(m_controller, XboxController.Button.kStart.value);
      JoystickButton lockBtn2 = new JoystickButton(m_controller, XboxController.Button.kBumperLeft.value);
      JoystickButton lockAlt = new JoystickButton(m_controller, XboxController.Button.kY.value);

      // lockBtn1.and(lockBtn2).whenActive(()->
      //   isLocked = !isLocked);
      lockAlt.whenPressed(() ->
        isLocked = !isLocked);
      
      
      
      JoystickButton compressorBtn = new JoystickButton(m_controller, XboxController.Button.kX.value);

      compressorBtn.whenPressed(() -> {
        m_compressor.start();
        System.out.println(m_compressor.enabled());
      });
      compressorBtn.whenReleased(() -> {
        m_compressor.stop();
        System.out.println(m_compressor.enabled());
      });
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

    return null;

  }
}
