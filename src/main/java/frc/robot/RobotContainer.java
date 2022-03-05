// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.LimelightArcCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.Wait;
import frc.lib.basesubsystem.LimelightSubsystem;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.basesubsystem.OakSubsystem;
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

  //oak subsystem
  // private OakSubsystem m_oak = new OakSubsystem("10.48.17.62", 5805);
  
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
  private ODN_CANCoder m_reachEncoder = new ODN_CANCoder(16);

  
  private MotorControllerSubsystem m_climberReach = new MotorControllerSubsystem(Constants.climberReachConstants);
  private ODN_CANCoder m_rotateEncoder = new ODN_CANCoder(15);

  //with set position
  // private ElevatorSubsystem m_climberReach = new MotorControllerSubsystem(Constants.climberReachConstants);

  // Controllers here:
  private XboxController m_controller = new XboxController(0);

  // Robot commands go here:
  // This command runs on autonomous
  private Command m_autoCommand = new SequentialCommandGroup(
    //new ShootCommand(m_shooter, m_indexer), 
    new DriveCommand(m_drive, -65, 86),
    new RotateCommand(m_drive, -22.5),
    new InstantCommand(()->m_intake.set(0.5), m_intake), 
    new DriveCommand(m_drive, 101, 50),
    new InstantCommand(()->m_intake.set(-0.1), m_intake), 
    new Wait(0.1),
    new InstantCommand(()->m_intake.set(0), m_intake), 
    new RotateCommand(m_drive, -157.5),
    new DriveCommand(m_drive, 105, 20),
    new ShooterCommand(m_shooterTop, m_shooterBottom, ShootCommand.shoot(25, 107.95, 21, 30, 2*60/(1/3 *2 *Math.PI)), true),
    new Wait(0.5),
    new InstantCommand(()->m_intake.set(0.5), m_intake),
    new Wait(4),
    new InstantCommand(()->{
      m_intake.set(0);
      m_shooterBottom.set(0);
      m_shooterTop.set(0);
    }, m_intake, m_shooterTop, m_shooterBottom),
    new DriveCommand(m_drive, -65, -25)


  );
  
  private double maintain = 0;

  private int topBall = 0;
  
  private int bottomBall = 0;

  private int intakeStatus = 0;

  

  // private int goal = 0;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    container = this;
    
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new RunCommand(() -> {
        m_drive.cartesianDriveAbsolute(modifyAxis(-m_controller.getLeftY()*m_controller.getLeftY()*m_controller.getLeftY()), 
          modifyAxis(-m_controller.getLeftX()*m_controller.getLeftX()*m_controller.getLeftX()),
          modifyAxis(-m_controller.getRightX()));
          
        m_limelight.setLED(false);
      },
      m_drive));


    //robot enabled (0/1), speed (number gets rounded to one decimal place), first button status (0/1), second button status (0/1), climb angle (0-30), extention length (0-1), intake status (0/1) 
    // m_oak.setDefaultCommand(new RunCommand(() -> {
    //   m_oak.writeData(String.format("%d %f %d %d %f %f %d", 1, m_shooterTop.getSpeed()/( 2*60/(1/3 *2 *Math.PI)), topBall, bottomBall, -m_rotateEncoder.getAbsolutePosition()+Constants.AutoConstants.kClimbRotationMinPosition, m_reachEncoder.getPosition()/Constants.AutoConstants.kClimbRotationMaxPosition, intakeStatus));
    // },
    // m_oak));

  }

  public double getAngle(){
    return m_limelight.getOffsetVertical();
  }

  boolean shooterOn = false;
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  

    //Intake in
    JoystickButton intakeIn = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    // intakeIn.whenPressed(new InstantCommand(()->m_intake.set(0.5), m_intake));
    // intakeIn.whenReleased(new InstantCommand(()->m_intake.set(0), m_intake));
    
    intakeIn.whenPressed(
      new ParallelCommandGroup(
        new InstantCommand(()->m_intake.set(0.5), m_intake), 
        new InstantCommand(()->{
          if(bottomBall == 1){
            topBall = 1;
          }
          else{
            bottomBall = 1;
          }
          intakeStatus = 1;
        }
    )));

    intakeIn.whenReleased(new InstantCommand(()->{
      m_intake.set(0);
      intakeStatus = 0;
    }, m_intake));
    
    //Intake out
    Trigger intakeOut = new Trigger(()->m_controller.getLeftTriggerAxis()>0.7);
    
    //intakeOut.whenPressed(new InstantCommand(()->m_intake.set(-0.5), m_intake));
    // intakeOut.whenReleased(new InstantCommand(()->m_intake.set(0), m_intake));
    
    intakeOut.whileActiveOnce(
      new ParallelCommandGroup(
      new InstantCommand(()->m_intake.set(-0.5), m_intake), 
      new InstantCommand(()-> {
        if(topBall == 1){
          topBall = 0;
        }
        else{
          bottomBall = 0;
        }
        intakeStatus = 1;

      }
    )));

    intakeOut.whenInactive(new InstantCommand(()->{
      m_intake.set(0);
      intakeStatus = 0;
    }, m_intake));
    
    
    

    //High shoot
    JoystickButton highShoot = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    
    highShoot.whileHeld(new ParallelCommandGroup(
      new ShooterCommand(m_shooterTop, m_shooterBottom, ShootCommand.shoot(25, 107.95, 21, 30, 2*60/(1/3 *2 *Math.PI)), true),
      new InstantCommand(()-> {
        if(topBall == 1 && bottomBall == 1){
          bottomBall = 0;
        }
        else{
          bottomBall = 0;
          topBall = 0;
        }
      }
    )));

    
    

    
    //Low shoot
    Trigger lowShoot = new Trigger(()->m_controller.getRightTriggerAxis()>0.7);
    lowShoot.whileActiveOnce(new ParallelCommandGroup(
      new ShooterCommand(m_shooterTop, m_shooterBottom, ShootCommand.shoot(25, 107.95, 21, 30, 2*60/(1/3 *2 *Math.PI)), false),
      new InstantCommand(()-> {
        if(topBall == 1 && bottomBall == 1){
          bottomBall = 0;
        }
        else{
          bottomBall = 0;
          topBall = 0;
        }
      }
    )));
    




    //Climber
    Trigger linearUp = new Trigger(()->m_controller.getPOV()==0);
    linearUp.whileActiveContinuous(new InstantCommand(()->{ 
      if(m_reachEncoder.getPosition()>Constants.AutoConstants.kClimbLinearMaxPosition){
        m_climberReach.set(0.75);


        //if using reach
        // if(goal<5900){
        //   goal+=5;
        //   m_climberReach.setGoalLocation(goal);
        // }
      }
      else{
        m_climberReach.set(0.05);
      
      }
      maintain = 0.05;
    }, m_climberReach));
    
    linearUp.whenInactive(new InstantCommand(()->m_climberReach.set(maintain), m_climberReach));

    Trigger linearDown = new Trigger(()->m_controller.getPOV()==180);
    linearDown.whileActiveContinuous(new InstantCommand(()->{ 
      if(m_reachEncoder.getPosition()<Constants.AutoConstants.kClimbLinearMinPosition){
        m_climberReach.set(-0.5);
        //if using reach
        // if(goal>=0){
        //   goal-=5;
        //   m_climberReach.setGoalLocation(goal);
        // }
        }
      else{
        m_climberReach.set(0.05);
        
      }
      maintain = 0.05;
       
  
    }, m_climberReach));

    linearDown.whenInactive(new InstantCommand(()->m_climberReach.set(maintain), m_climberReach));

    JoystickButton lockReach = new JoystickButton(m_controller, XboxController.Button.kA.value);
    lockReach.whenPressed(new InstantCommand(()->m_climberReach.set(-0.1),m_climberReach));
    


    
    


    Trigger rotateForward = new Trigger(()->m_controller.getPOV()==270);
    rotateForward.whileActiveContinuous(new InstantCommand(()->{ 
      if(m_rotateEncoder.getAbsolutePosition()>Constants.AutoConstants.kClimbRotationMaxPosition || m_rotateEncoder.getAbsolutePosition()<180){
        m_climberRotate.set(0.5);
      }
      else{
        m_climberRotate.set(-0.05);
      }
    }, m_climberRotate));

    rotateForward.whenInactive(new InstantCommand(()->m_climberRotate.set(0), m_climberRotate));


    Trigger rotateBackward = new Trigger(()->m_controller.getPOV()==90);
    rotateBackward.whileActiveContinuous(new InstantCommand(()->{ 
      if(m_rotateEncoder.getAbsolutePosition()<Constants.AutoConstants.kClimbRotationMinPosition && m_rotateEncoder.getAbsolutePosition()>180 ){
        m_climberRotate.set(-0.5);
      }
      else{
        m_climberRotate.set(0.05);
      }
    }, m_climberRotate));

    rotateBackward.whenInactive(new InstantCommand(()->m_climberRotate.set(0), m_climberRotate));

    
    JoystickButton lockRotate = new JoystickButton(m_controller, XboxController.Button.kX.value);
    lockRotate.whenPressed(new InstantCommand(()->m_climberRotate.set(-0.15),m_climberReach));
    



    //Align
    // JoystickButton align = new JoystickButton(m_controller, XboxController.Button.kA.value);
    // align.toggleWhenPressed(new SequentialCommandGroup(
    //   new InstantCommand(()->m_limelight.setLED(true),m_limelight),
    //   new Wait(),
    //   new LimelightArcCommand(m_drive, m_limelight, LimelightSubsystem.linearAttenuation(27), ShootCommand.hood(55.0, 70.0, 25.0, 107.95), m_controller)));
   
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
