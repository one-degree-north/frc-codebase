// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.basesubsystem.LimelightSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;
import frc.robot.commands.ElevatorHeightCommand;
import frc.robot.commands.IndexerContinueCommand;
import frc.robot.commands.IndexerRunCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static RobotContainer container;
  // Robot subsystems here:
  // private SwerveDriveSubsystem m_drive = new SwerveDriveSubsystem(Constants.swerveConstants);
  private LimelightSubsystem m_limelight = new LimelightSubsystem();
  private ShooterSubsystem m_shooter = new ShooterSubsystem(Constants.shooterConstants);
  private IndexerSubsystem m_indexer = new IndexerSubsystem(Constants.indexerConstants);
  private IntakeSubsystem m_intake = new IntakeSubsystem(Constants.intakeConstants);
  private ClimbSubsystem m_climb = new ClimbSubsystem(Constants.climbConstants);

  // Controllers here:
  private XboxController m_controller = new XboxController(0);

  // Robot commands go here:


  // This command runs on autonomous
  private Command m_autoCommand = null;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    container = this;

    // Configure the button bindings
    configureButtonBindings();

    // m_drive.setDefaultCommand(new RunCommand(() -> {
    //     m_drive.cartesianDriveAbsolute(modifyAxis(m_controller.getLeftY()), 
    //       modifyAxis(m_controller.getLeftX()),
    //       modifyAxis(m_controller.getRightX()));
    //   },
    //   m_drive));
    
    m_climb.disable();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Trigger intakeToggle = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    Trigger intakeRun = new Trigger(()->m_controller.getLeftTriggerAxis()>0.5);
    Trigger manualShooterRev = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    Trigger shootBall = new Trigger(()->m_controller.getRightTriggerAxis()>0.5);
    Trigger ballAtEntrance = m_indexer.ballAtEntrance();
    Trigger ballAtExit = m_indexer.ballAtExit();

    //drop or raise intake
    intakeToggle.whenActive(() -> m_intake.toggle(), m_intake);

    //turn on or off the intake
    intakeRun.whenActive(() -> m_intake.toggleRun(), m_intake);

    //if a ball enters the indexer, rev up the shooter
    ballAtEntrance.whenActive(() -> m_shooter.on(), m_shooter);

    //shoot ball
    shootBall.whenActive(new IndexerContinueCommand(m_indexer));

    //if manual shooter rev button is pressed, toggle whether the shooter is revving or off
    manualShooterRev.whenActive(() -> m_shooter.toggle(), m_shooter);

    //if ball is at entrance of indexer and no ball is at the exit, move it to the end
    ballAtEntrance.and(ballAtExit.negate()).whenActive(new IndexerRunCommand(m_indexer));

    //climbing stuff: figure this out later
    JoystickButton climb = new JoystickButton(m_controller, XboxController.Button.kY.value);
    climb.whenPressed(new SequentialCommandGroup(
      new InstantCommand(()->m_climb.enable(), m_climb),
      new ElevatorHeightCommand(m_climb, ClimbSubsystem.TOP),
      //Drive robot back a little
      new ElevatorHeightCommand(m_climb, ClimbSubsystem.BOTTOM),
      new InstantCommand(()->m_climb.contractRotation(), m_climb),
      new ElevatorHeightCommand(m_climb, ClimbSubsystem.TRANSFER),
      new InstantCommand(()->m_climb.extendRotation(), m_climb),
      new ElevatorHeightCommand(m_climb, ClimbSubsystem.TOP)
    ));
    
  }
  
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
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

  public IndexerSubsystem getIndexer() {
    return m_indexer;
  }

  public IntakeSubsystem getIntake() {
    return m_intake;
  }

  public ShooterSubsystem getShooter() {
    return m_shooter;
  }

  public SwerveDriveSubsystem getDrivebase() {
    return null;
  }

  public LimelightSubsystem getLimelight() {
    return m_limelight;
  }

  public XboxController getJoystick() {
    return m_controller;
  }
}
