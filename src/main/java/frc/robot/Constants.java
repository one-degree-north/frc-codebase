// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.lib.motorcontroller.ODN_MotorControllerGroup;
import frc.lib.motorcontroller.ODN_SparkMax;
import frc.lib.motorcontroller.ODN_TalonFX;
import frc.lib.motorcontroller.ODN_SparkMax.MotorType;
import frc.lib.sensor.ODN_Adafruit164Sensor;
import frc.lib.sensor.ODN_ColorSensor;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.basesubsystem.PneumaticSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;
import frc.lib.gyro.ODN_AHRS;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double DIST_TO_SHOOT_FROM = 0;

    public static SwerveDriveSubsystem.Constants swerveConstants = new SwerveDriveSubsystem.Constants();
    static {
        swerveConstants.DRIVETRAIN_TRACKWIDTH_METERS = 20.25*2.54/100;
        swerveConstants.DRIVETRAIN_WHEELBASE_METERS = 25.25*2.534/100;
    
        swerveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
        swerveConstants.FRONT_LEFT_MODULE_STEER_MOTOR = 1;
        swerveConstants.FRONT_LEFT_MODULE_STEER_ENCODER = 18;
        swerveConstants.FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(285.5);
    
        swerveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6;
        swerveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
        swerveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER = 20;
        swerveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(293.2);
    
        swerveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
        swerveConstants.BACK_LEFT_MODULE_STEER_MOTOR = 3;
        swerveConstants.BACK_LEFT_MODULE_STEER_ENCODER = 19;
        swerveConstants.BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(152.4);

        swerveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
        swerveConstants.BACK_RIGHT_MODULE_STEER_MOTOR = 7;
        swerveConstants.BACK_RIGHT_MODULE_STEER_ENCODER = 21;
        swerveConstants.BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(293.38);

        swerveConstants.gyro = new ODN_AHRS();
    }

    // -----------------------------------------------------------------------
    // --------- SHOOTER -----------------------------------------------------
    // -----------------------------------------------------------------------

    public static ShooterSubsystem.Constants shooterConstants = new ShooterSubsystem.Constants();
    static {
        shooterConstants.motor = new MotorControllerSubsystem.Constants();
        shooterConstants.motor.motor = new ODN_MotorControllerGroup(
            new ODN_TalonFX(9).setInverted(true),
            new ODN_TalonFX(10)
        );
        shooterConstants.shoot_speed = 3000;
    }
    
    // -----------------------------------------------------------------------
    // --------- INDEXER -----------------------------------------------------
    // -----------------------------------------------------------------------

    public static IndexerSubsystem.Constants indexerConstants = new IndexerSubsystem.Constants();
    static {
        
        indexerConstants.indexer = new MotorControllerSubsystem.Constants();
        indexerConstants.indexer.motor = new ODN_TalonFX(14);
        indexerConstants.feeder = new MotorControllerSubsystem.Constants();
        indexerConstants.feeder.motor = new ODN_SparkMax(15, MotorType.brushless);
        indexerConstants.color = new ODN_ColorSensor(I2C.Port.kMXP);
        indexerConstants.exit_sensor = new DigitalInput(2);
        indexerConstants.encoder = indexerConstants.feeder.motor.getEncoder();
    }

    // -----------------------------------------------------------------------
    // --------- INTAKE ------------------------------------------------------
    // -----------------------------------------------------------------------

    public static IntakeSubsystem.Constants intakeConstants = new IntakeSubsystem.Constants();
    static {
        intakeConstants.motor = new MotorControllerSubsystem.Constants();
        intakeConstants.motor.motor = new ODN_TalonFX(13);

        intakeConstants.pneumatics = new PneumaticSubsystem.Constants();
        intakeConstants.pneumatics.solenoids = new DoubleSolenoid[]{
            new DoubleSolenoid(17, PneumaticsModuleType.CTREPCM, 6, 7)
        };
    }

    // -----------------------------------------------------------------------
    // --------- CLIMBER -----------------------------------------------------
    // -----------------------------------------------------------------------

    public static ClimbSubsystem.Constants climbConstants = new ClimbSubsystem.Constants();
    static {
        climbConstants.enable_climber = new PneumaticSubsystem.Constants();
        climbConstants.enable_climber.solenoids = new DoubleSolenoid[] {
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3)
        };
        climbConstants.rotation = new PneumaticSubsystem.Constants();
        climbConstants.rotation.solenoids = new DoubleSolenoid[] {
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5)
        };
        climbConstants.motor = new MotorControllerSubsystem.Constants();
        climbConstants.motor.motor = new ODN_MotorControllerGroup(new ODN_TalonFX(22), new ODN_TalonFX(23).setInverted(true));
    }

    // -----------------------------------------------------------------------
    // --------- AUTO CONSTANTS ----------------------------------------------
    // -----------------------------------------------------------------------

    public static final class AutoConstants {
        public static double kMaxSpeedMetersPerSecond = 1;
        public static double kMaxAccelerationMetersPerSecondSquared = 0.8;
        
        public static final double kMaxAngularSpeedRadiansPerSecond = 40;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 40;

        // These numbers must be correctly calculated
        public static double kRamseteB = 1;
        public static double kRamseteZeta = 1;
		public static double kPYController = 1;
		public static double kPXController = 1;
		public static double kPThetaController = 1;
    }

}
