// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.lib.basesubsystem.ArmSubsystem;
import frc.lib.basesubsystem.PneumaticSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;
import frc.lib.basesubsystem.TankDriveSubsystem;
import frc.lib.encoder.ODN_NullEncoder;
import frc.lib.gyro.ODN_AHRS;
import frc.lib.gyro.ODN_NullGyro;
import frc.lib.motorcontroller.ODN_MotorController;
import frc.lib.motorcontroller.ODN_MotorControllerGroup;
import frc.lib.motorcontroller.ODN_SparkMax;
import frc.lib.motorcontroller.ODN_TalonFX;
import frc.robot.subsystems.TwoLinkArmSubsystem;

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
    public static TankDriveSubsystem.Constants tankDriveConstants = new TankDriveSubsystem.Constants();
    static {
        tankDriveConstants.left = new ODN_MotorControllerGroup(new ODN_SparkMax(0, frc.lib.motorcontroller.ODN_SparkMax.MotorType.brushed), new ODN_SparkMax(1, frc.lib.motorcontroller.ODN_SparkMax.MotorType.brushed));
        tankDriveConstants.right = new ODN_MotorControllerGroup(new ODN_SparkMax(2, frc.lib.motorcontroller.ODN_SparkMax.MotorType.brushed), new ODN_SparkMax(3, frc.lib.motorcontroller.ODN_SparkMax.MotorType.brushed));
        tankDriveConstants.leftEncoder = new ODN_NullEncoder();
        tankDriveConstants.rightEncoder = new ODN_NullEncoder();
        tankDriveConstants.gyro = new ODN_NullGyro();
        tankDriveConstants.leftEncoderFactor = 0;
        tankDriveConstants.rightEncoderFactor = 0;

        tankDriveConstants.ksVolts = 1;
        tankDriveConstants.kvVoltSecondsPerMeter = 0;
        tankDriveConstants.kaVoltSecondsSquaredPerMeter = 0;
        tankDriveConstants.kDriveKinematics = new DifferentialDriveKinematics(0.55);

    }

    public static TwoLinkArmSubsystem.Constants twoLinkArmConstants = new TwoLinkArmSubsystem.Constants();
    static {
        // First arm link (closest to base)
        twoLinkArmConstants.armlink1 = new ArmSubsystem.Constants();

        twoLinkArmConstants.armlink1.motor = new ODN_TalonFX(0); // set can id
        twoLinkArmConstants.armlink1.encoder = twoLinkArmConstants.armlink1.motor.getEncoder(); // gets built in encoder from talonfx
        twoLinkArmConstants.armlink1.encoderFactor = 0.0; // talonfx encoder output to meters conversion
        twoLinkArmConstants.armlink1.maxVelocity = 0.0; // max velocity
        twoLinkArmConstants.armlink1.maxAcceleration = 0.0; // max accel

        // PID constants
        twoLinkArmConstants.armlink1.kp = 0.0;
        twoLinkArmConstants.armlink1.ki = 0.0;
        twoLinkArmConstants.armlink1.kd = 0.0;

        // Feedforward constants
        twoLinkArmConstants.armlink1.ks = 0.0;
        twoLinkArmConstants.armlink1.kcos = 0.0;
        twoLinkArmConstants.armlink1.kv = 0.0;

        twoLinkArmConstants.armlink1.length = 0.0; // arm length from hinge (i think, need to crosscheck)

        // Second arm link (far from base)

        twoLinkArmConstants.armlink2 = new ArmSubsystem.Constants();

        twoLinkArmConstants.armlink2.motor = new ODN_TalonFX(1); // set can id
        twoLinkArmConstants.armlink2.encoder = twoLinkArmConstants.armlink2.motor.getEncoder(); // gets built in encoder from talonfx
        twoLinkArmConstants.armlink2.encoderFactor = 0.0; // talonfx encoder output to meters conversion
        twoLinkArmConstants.armlink2.maxVelocity = 0.0; // max velocity
        twoLinkArmConstants.armlink2.maxAcceleration = 0.0; // max accel

        // PID constants
        twoLinkArmConstants.armlink2.kp = 0.0;
        twoLinkArmConstants.armlink2.ki = 0.0;
        twoLinkArmConstants.armlink2.kd = 0.0;

        // Feedforward constants
        twoLinkArmConstants.armlink2.ks = 0.0;
        twoLinkArmConstants.armlink2.kcos = 0.0;
        twoLinkArmConstants.armlink2.kv = 0.0;

        twoLinkArmConstants.armlink2.length = 0.0; // arm length from hinge (i think, need to crosscheck)
    }


    // public static SwerveDriveSubsystem.Constants swerveConstants = new SwerveDriveSubsystem.Constants();
    // static {
    //     swerveConstants.DRIVETRAIN_TRACKWIDTH_METERS = 0.47;
    //     swerveConstants.DRIVETRAIN_WHEELBASE_METERS = 0.47;
    
    //     swerveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    //     swerveConstants.FRONT_LEFT_MODULE_STEER_MOTOR = 1;
    //     swerveConstants.FRONT_LEFT_MODULE_STEER_ENCODER = 12;
    //     swerveConstants.FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(7.471);
    
    //     swerveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6;
    //     swerveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
    //     swerveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
    //     swerveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(136.055-180);
    
    //     swerveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR = 4; 
    //     swerveConstants.BACK_LEFT_MODULE_STEER_MOTOR = 3;
    //     swerveConstants.BACK_LEFT_MODULE_STEER_ENCODER = 9;
    //     swerveConstants.BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(71.542-180);

    //     swerveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
    //     swerveConstants.BACK_RIGHT_MODULE_STEER_MOTOR = 7;
    //     swerveConstants.BACK_RIGHT_MODULE_STEER_ENCODER = 10;
    //     swerveConstants.BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-21.533);

    //     swerveConstants.gyro = new ODN_AHRS();
    // }

    public static final class AutoConstants {
        public static double kMaxSpeedMetersPerSecond = 1;
        public static double kMaxAccelerationMetersPerSecondSquared = 1;
        
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
