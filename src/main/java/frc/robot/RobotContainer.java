// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Set;

import javax.print.attribute.standard.Compression;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.basesubsystem.LimelightSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorHeightCommand;
import frc.robot.commands.IndexerContinueCommand;
import frc.robot.commands.IndexerRunCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.commands.Wait;
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
  boolean r;
  public static RobotContainer container;
  // Robot subsystems here:
  private SwerveDriveSubsystem m_drive = new SwerveDriveSubsystem(Constants.swerveConstants);
  private ShooterSubsystem m_shooter = new ShooterSubsystem(Constants.shooterConstants);
  private IndexerSubsystem m_indexer = new IndexerSubsystem(Constants.indexerConstants);
  private IntakeSubsystem m_intake = new IntakeSubsystem(Constants.intakeConstants);

  private ClimbSubsystem m_climb = new ClimbSubsystem(Constants.climbConstants);

  // Controllers here:
  private XboxController m_controller = new XboxController(0);
  private Compressor compressor = new Compressor(17, PneumaticsModuleType.CTREPCM);

  // Robot commands go here:
  public Command getIntakeToggleCommand() {
    return new SequentialCommandGroup(
    new InstantCommand(()->{
      r = m_intake.isRunning();
      if(r) {
        m_intake.off();
      } else {
        m_intake.drop();
      }
    }),
    new Wait(),
    new InstantCommand(()->{
      if(r) {
        m_intake.raise();
      } else {
        m_intake.on();
      }
    })
    
  );
  }

  // This command runs on autonomous
  private Command m_autoCommand = new SequentialCommandGroup(
    //new ShootCommand(m_shooter, m_indexer), 
    new DriveCommand(m_drive, 90+25, 86),
    new RotateCommand(m_drive, 157.5),
    getIntakeToggleCommand(),
    new DriveCommand(m_drive, 270+11, 50),
    getIntakeToggleCommand(),
    new RotateCommand(m_drive, -157.5),
    new DriveCommand(m_drive, -90+25, 138),
    new DriveCommand(m_drive, 90+25, 138)

    );


  private double lastRightXInput;
  private double lastLeftXInput;
  private double lastLeftYInput;
  private double lastInputTime;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    container = this;

    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new RunCommand(() -> {
      double t = System.currentTimeMillis();
      double dt = t - lastInputTime;
      double diffRightX = (m_controller.getRightX() - lastRightXInput)/dt;
      double diffLeftX = (m_controller.getLeftX()- lastLeftXInput)/dt;
      double diffLeftY = (m_controller.getLeftX()- lastLeftYInput)/dt;

      if(diffRightX > 0.5)
        diffRightX = 0.5;
      else if(diffRightX < -0.5)
        diffRightX = -0.5;

      if(diffLeftX > 0.5)
        diffLeftX = 0.5;
      else if(diffLeftX < -0.5)
        diffLeftX = -0.5;

      if(diffLeftY > 0.5)
        diffLeftY = 0.5;
      else if(diffLeftY < -0.5)
        diffLeftY = -0.5;
      
      double newRightX = lastRightXInput + diffRightX;
      double newLeftX = lastLeftXInput + diffLeftX;
      double newLeftY = lastLeftYInput + diffLeftY;
      
        m_drive.cartesianDriveRelative(modifyAxis(newLeftY), 
          modifyAxis(newLeftX),
          modifyAxis(newRightX));
        lastRightXInput = newRightX;
        lastLeftXInput = newLeftX;
        lastLeftYInput = newLeftY;
        lastInputTime = t;
      },
      m_drive));

    m_indexer.setDefaultCommand(new RunCommand(()->{
      //m_indexer.onboth();
    }, m_indexer));

    //compressor.disable();
    
    //m_climb.disable();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
    Trigger manualShooterRev = new JoystickButton(m_controller, XboxController.Button.kA.value);
    Trigger shootBall = new Trigger(()->m_controller.getRightTriggerAxis()>0.5);
    Trigger intakeToggle3 = new Trigger(()->m_controller.getLeftTriggerAxis()>0.5);
    manualShooterRev.whenActive(() -> m_shooter.toggle(), m_shooter);
    shootBall.whenActive(new IndexerContinueCommand(m_indexer));
    Trigger intakeToggle = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    intakeToggle.whenActive(() -> m_indexer.onboth(), m_intake);
    Trigger intakeToggle2 = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    intakeToggle2.whenActive(() -> m_indexer.off(), m_intake);
    intakeToggle3.whenActive(() -> m_intake.toggleRun(), m_intake);
`   */
    
    Trigger intakeRun = new Trigger(()->m_controller.getLeftTriggerAxis()>0.5);
    Trigger shootBall = new Trigger(()->m_controller.getRightTriggerAxis()>0.5);
    

    Trigger t = new JoystickButton(m_controller, XboxController.Button.kB.value);
    t.whenActive(()->{
      if(compressor.enabled()) {
        compressor.disable();
      } else {
        compressor.enableDigital();
      }
    });
    
    //drop or release
    intakeRun.whenActive(getIntakeToggleCommand());
    intakeRun.whenInactive(getIntakeToggleCommand());

    m_indexer.ballAtExit().whileActiveOnce(new RunCommand(()->m_indexer.off(), m_indexer));

    //shoot ball
    shootBall.whenActive(new IndexerContinueCommand(m_indexer));

    //manual climber
    Trigger climberUp = new  Trigger(()->m_controller.getPOV()==0);
    Trigger climberDown = new  Trigger(()->m_controller.getPOV()==180);
    Trigger climberRight = new  Trigger(()->m_controller.getPOV()==90);
    Trigger climberLeft = new  Trigger(()->m_controller.getPOV()==270);
    Trigger climberToggle = new JoystickButton(m_controller, XboxController.Button.kY.value);
    double climbSpeed = 0.1;
    
    climberToggle.whenActive(new InstantCommand(()-> m_climb.toggle()));
    climberUp.whileActiveContinuous(new InstantCommand(()-> m_climb.setMotor(climbSpeed), m_climb));
    climberDown.whileActiveContinuous(new InstantCommand(()-> m_climb.setMotor(-climbSpeed), m_climb));
    climberRight.whenActive(new InstantCommand(()-> m_climb.extendRotation(), m_climb));
    climberLeft.whenActive(new InstantCommand(()-> m_climb.contractRotation(), m_climb));

    //climbing stuff: figure this out later
    // JoystickButton climb = new JoystickButton(m_controller, XboxController.Button.kY.value);
    // climb.whenPressed(new SequentialCommandGroup(
    //   new InstantCommand(()->m_climb.enable(), m_climb),
    //   new ElevatorHeightCommand(m_climb, ClimbSubsystem.TOP),
    //   new DriveBackCommand(m_drive),
    //   new ElevatorHeightCommand(m_climb, ClimbSubsystem.BOTTOM),
    //   new InstantCommand(()->m_climb.contractRotation(), m_climb),
    //   new ElevatorHeightCommand(m_climb, ClimbSubsystem.TRANSFER),
    //   new InstantCommand(()->m_climb.extendRotation(), m_climb),
    //   new ElevatorHeightCommand(m_climb, ClimbSubsystem.TOP)
    // ));
    
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
    return m_drive;
  }

  public XboxController getJoystick() {
    return m_controller;
  }
}