// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import frc.lib.encoder.NullEncoder;
import frc.lib.encoder.ODN_CANCoder;
import frc.lib.motorcontroller.MotorControllerGroup;
import frc.lib.motorcontroller.ODN_SparkMax;
import frc.lib.motorcontroller.ODN_TalonFX;
import frc.lib.motorcontroller.ODN_VictorSPX;
import frc.lib.motorcontroller.ODN_SparkMax.MotorType;
import frc.robot.subsystems.SwerveDriveSubsystem;

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


    public static SwerveDriveSubsystem.Constants swerveConstants = new SwerveDriveSubsystem.Constants();
    static {
        swerveConstants.DRIVETRAIN_TRACKWIDTH_METERS = 0.47; // FIXME Measure and set trackwidth
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        swerveConstants.DRIVETRAIN_WHEELBASE_METERS = 0.47; // FIXME Measure and set wheelbase
    
        swerveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR = 2; // FIXME Set front left module drive motor ID
        swerveConstants.FRONT_LEFT_MODULE_STEER_MOTOR = 1; // FIXME Set front left module steer motor ID
        swerveConstants.FRONT_LEFT_MODULE_STEER_ENCODER = 12; // FIXME Set front left steer encoder ID
        swerveConstants.FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(7.471); // FIXME Measure and set front left steer offset
    
        swerveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6; // FIXME Set front right drive motor ID
        swerveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR = 5; // FIXME Set front right steer motor ID
        swerveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER = 11; // FIXME Set front right steer encoder ID
        swerveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(136.055-180); // FIXME Measure and set front right steer offset
    
        swerveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR = 4; // FIXME Set back left drive motor ID
        swerveConstants.BACK_LEFT_MODULE_STEER_MOTOR = 3; // FIXME Set back left steer motor ID
        swerveConstants.BACK_LEFT_MODULE_STEER_ENCODER = 9; // FIXME Set back left steer encoder ID
        swerveConstants.BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(71.542-180); // FIXME Measure and set back left steer offset
    
        swerveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR = 8; // FIXME Set back right drive motor ID
        swerveConstants.BACK_RIGHT_MODULE_STEER_MOTOR = 7; // FIXME Set back right steer motor ID
        swerveConstants.BACK_RIGHT_MODULE_STEER_ENCODER = 10; // FIXME Set back right steer encoder ID
        swerveConstants.BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-21.533); // FIXME Measure and set back right steer offset

        
    }

    public static int kControllerPort = 0;
    
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
		public static Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
		public static double kPThetaController = 1;
    }

}
