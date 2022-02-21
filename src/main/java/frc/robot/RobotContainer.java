// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.LimelightArcCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.HoodSubsystem;
import frc.lib.basesubsystem.ElevatorSubsystem;
import frc.lib.basesubsystem.LimelightSubsystem;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;
import frc.lib.encoder.ODN_CANCoder;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


  public static RobotContainer container;
  // Robot subsystems here:

  //Drivebase (swerve)
  private SwerveDriveSubsystem m_drive = new SwerveDriveSubsystem(Constants.swerveConstants);
  
  //Limelight
  private LimelightSubsystem m_limelight = new LimelightSubsystem();

  //Intake and Indexer
  private MotorControllerSubsystem m_intake = new MotorControllerSubsystem(Constants.intakeConstants);

  //Shooter
  private MotorControllerSubsystem m_shooterTop = new MotorControllerSubsystem(Constants.shooterTopConstants);
  private MotorControllerSubsystem m_shooterBottom = new MotorControllerSubsystem(Constants.shooterBottomConstants);

  //Hood 
  
  //Climber
  private MotorControllerSubsystem m_climberRotate = new MotorControllerSubsystem(Constants.climberRotateConstants);
  private ODN_CANCoder m_reachEncoder = new ODN_CANCoder(15);

  
  private MotorControllerSubsystem m_climberReach = new MotorControllerSubsystem(Constants.climberReachConstants);
  private ODN_CANCoder m_rotateEncoder = new ODN_CANCoder(16);

  //with set position
  // private ElevatorSubsystem m_climberReach = new MotorControllerSubsystem(Constants.climberReachConstants);

  // Controllers here:
  private XboxController m_controller = new XboxController(0);

  // Robot commands go here:
  // This command runs on autonomous
  private Command m_autoCommand = null;
  
  private int maintain = -10;

  // private int goal = 0;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    container = this;
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new RunCommand(() -> {
        m_drive.cartesianDriveAbsolute(modifyAxis(m_controller.getLeftY()), 
          modifyAxis(m_controller.getLeftX()),
          modifyAxis(m_controller.getRightX()));
      },
      m_drive));

    m_climberRotate.setDefaultCommand(new RunCommand(() -> {
      m_climberRotate.setSpeed(0);
    },
    m_climberRotate));

    m_climberReach.setDefaultCommand(new RunCommand(() -> {
      m_climberReach.setSpeed(maintain);
    },
    m_climberReach));
  
  }

  public double getAngle(){
    return m_limelight.getOffsetVertical();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


    //Intake in
    JoystickButton intakeIn = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    intakeIn.whenPressed(new IndexerCommand(m_intake, true));
    //Intake out
    Trigger intakeOut = new Trigger(()->m_controller.getLeftTriggerAxis()>0.7);
    intakeOut.whenActive(new IndexerCommand(m_intake, false));
    
    

    //High shoot
    JoystickButton highShoot = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    highShoot.whenPressed(new ShootCommand(m_drive, m_limelight, m_intake, m_shooterTop, m_shooterBottom, m_controller));
    //Low shoot
    Trigger lowShoot = new Trigger(()->m_controller.getRightTriggerAxis()>0.7);
    lowShoot.whenActive(new ShooterCommand(m_shooterTop, m_shooterBottom, ShootCommand.shoot(25, 107.95, 21, 30, 5), false));
    

    //Climber
    Trigger linearUp = new Trigger(()->m_controller.getPOV()==0);
    linearUp.whenActive(new InstantCommand(()->{ 
      if(m_reachEncoder.getPosition()<Constants.AutoConstants.kClimbLinearMaxPosition){
        m_climberReach.setSpeed(-5900/2);
        maintain = -10;

        
        //if using reach
        // if(goal<5900){
        //   goal+=5;
        //   m_climberReach.setGoalLocation(goal);
        // }
      }
    }));
    Trigger linearDown = new Trigger(()->m_controller.getPOV()==180);
    linearDown.whenActive(new InstantCommand(()->{ 
      if(m_reachEncoder.getPosition()>Constants.AutoConstants.kClimbLinearMinPosition){
        m_climberReach.setSpeed(5900/2);
        maintain = 1000;

        //if using reach
        // if(goal>=0){
        //   goal-=5;
        //   m_climberReach.setGoalLocation(goal);
        // }
      }
    }));
    


    Trigger rotateForward = new Trigger(()->m_controller.getPOV()==90);
    rotateForward.whenActive(new InstantCommand(()->{ 
      if(m_rotateEncoder.getAbsolutePosition()>Constants.AutoConstants.kClimbRotationMaxPosition){
        m_climberRotate.setSpeed(5800/2);
      }
    }));
    Trigger rotateBackward = new Trigger(()->m_controller.getPOV()==270);
    rotateBackward.whenActive(new InstantCommand(()->{ 
      if(m_rotateEncoder.getAbsolutePosition()<Constants.AutoConstants.kClimbRotationMinPosition){
        m_climberRotate.setSpeed(-5800/2);
      }
    }));


    //Align
    JoystickButton align = new JoystickButton(m_controller, XboxController.Button.kA.value);
    align.toggleWhenPressed(new LimelightArcCommand(m_drive, m_limelight, LimelightSubsystem.linearAttenuation(27), ShootCommand.hood(55.0, 70.0, 25.0, 107.95), m_controller));
   
  }
  
  
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
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
    // deadband
    value = deadband(value, 0.05);

    // square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
