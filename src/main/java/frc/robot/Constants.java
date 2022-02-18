// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.lib.encoder.ODN_CANCoder;
import frc.lib.gyro.ODN_AHRS;
import frc.lib.motorcontroller.ODN_MotorControllerGroup;
import frc.lib.motorcontroller.ODN_SparkMax;
import frc.lib.motorcontroller.ODN_TalonFX;
import frc.lib.motorcontroller.ODN_SparkMax.Type;
import frc.robot.subsystems.HoodSubsystem;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;

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

    //drivebase
    public static SwerveDriveSubsystem.Constants swerveConstants = new SwerveDriveSubsystem.Constants();
    static {
        swerveConstants.DRIVETRAIN_TRACKWIDTH_METERS = 0.68;
        swerveConstants.DRIVETRAIN_WHEELBASE_METERS = 0.68;
    
        swerveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR = 6; //
        swerveConstants.FRONT_LEFT_MODULE_STEER_MOTOR = 5; //
        swerveConstants.FRONT_LEFT_MODULE_STEER_ENCODER = 10;
        swerveConstants.FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(7.471);
    
        swerveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; //
        swerveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR = 3; //
        swerveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
        swerveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(136.055-180);
    
        swerveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR = 8; //
        swerveConstants.BACK_LEFT_MODULE_STEER_MOTOR = 7; //
        swerveConstants.BACK_LEFT_MODULE_STEER_ENCODER = 9;
        swerveConstants.BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(71.542-180);

        swerveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR = 2; //
        swerveConstants.BACK_RIGHT_MODULE_STEER_MOTOR = 1; //
        swerveConstants.BACK_RIGHT_MODULE_STEER_ENCODER = 12;
        swerveConstants.BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-21.533);

        swerveConstants.gyro = new ODN_AHRS();
    }


    //intake
    public static MotorControllerSubsystem.Constants intakeConstants = new MotorControllerSubsystem.Constants();

    static {
        intakeConstants.motor = new ODN_MotorControllerGroup(new ODN_SparkMax(41, Type.brushless), new ODN_SparkMax(42, Type.brushless));
    }

    //shooter

    public static MotorControllerSubsystem.Constants shooterTopConstants = new MotorControllerSubsystem.Constants();
    public static MotorControllerSubsystem.Constants shooterBottomConstants = new MotorControllerSubsystem.Constants();

    static {
        shooterTopConstants.motor = new ODN_TalonFX(13);
        shooterBottomConstants.motor = new ODN_TalonFX(14);
    }

    //climber

    public static MotorControllerSubsystem.Constants climberRotateConstants = new MotorControllerSubsystem.Constants();
    public static MotorControllerSubsystem.Constants climberReachConstants = new MotorControllerSubsystem.Constants();

    static {
        climberRotateConstants.motor = new ODN_TalonFX(13);
        climberReachConstants.motor = new ODN_TalonFX(14);
        
    }    

    //hood

    public static HoodSubsystem.Constants hoodConstants = new HoodSubsystem.Constants();
    static {
        hoodConstants.minValue = 55;
        hoodConstants.maxValue = 70;
        hoodConstants.motor = new ODN_TalonFX(18);
        hoodConstants.encoder = new ODN_CANCoder(19);
        hoodConstants.conversion = 1/100;
        hoodConstants.kp=1;
        hoodConstants.ki=0;
        hoodConstants.kd=0;
        hoodConstants.maxVelocity=2;
        hoodConstants.maxVelocity=5;



        
    }

    public static final class AutoConstants {
        public static double kMaxSpeedMetersPerSecond = 1;
        public static double kMaxAccelerationMetersPerSecondSquared = 1;
        
        public static final double kMaxAngularSpeedRadiansPerSecond = 40;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 40;

        public static final double kClimbLinearMaxPosition = 5900;
        public static final double kClimbLinearMinPosition = 0;
        public static final double kClimbRotationMaxPosition = 120;
        public static final double kClimbRotationMinPosition = -80;

        
        // These numbers must be correctly calculated
        public static double kRamseteB = 1;
        public static double kRamseteZeta = 1;
		public static double kPYController = 1;
		public static double kPXController = 1;
		public static double kPThetaController = 1;
    }

}
