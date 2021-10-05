// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import frc.lib.encoder.ODN_CANCoder;
import frc.lib.motorcontroller.ODN_SparkMax;
import frc.lib.motorcontroller.ODN_TalonSRX;
import frc.lib.motorcontroller.ODN_SparkMax.MotorType;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDriveSubsystem;

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

    public static TankDriveSubsystem.Constants drive_constants = new TankDriveSubsystem.Constants();
    static {
        drive_constants.frontLeft = new ODN_TalonSRX(1);
        drive_constants.backLeft = new ODN_TalonSRX(2);
        drive_constants.frontRight = new ODN_TalonSRX(3);
        drive_constants.backRight = new ODN_TalonSRX(4);
        drive_constants.leftEncoder = drive_constants.frontLeft.getEncoder();
        drive_constants.rightEncoder = drive_constants.frontRight.getEncoder();
        drive_constants.id_gyro = 5;

        // These numbers must be correctly calculated
        drive_constants.ksVolts = 1;
        drive_constants.kvVoltSecondsPerMeter = 0;
        drive_constants.kaVoltSecondsSquaredPerMeter = 0;

        drive_constants.kDriveKinematics = new DifferentialDriveKinematics(0.1);
    }

    public static SwerveDriveSubsystem.Constants swerve_constants = new SwerveDriveSubsystem.Constants();
    static {

        swerve_constants.wheelDiameterMeters = 2;
        
		swerve_constants.maxSpeedMetersPerSecond = 2;

		swerve_constants.gyroReversed = true;

		swerve_constants.id_md_fl = new ODN_SparkMax(1, MotorType.brushed);
		swerve_constants.id_md_bl = new ODN_SparkMax(2, MotorType.brushed);
		swerve_constants.id_md_fr = new ODN_SparkMax(3, MotorType.brushed);
        swerve_constants.id_md_br = new ODN_SparkMax(4, MotorType.brushed);
        
		swerve_constants.id_mt_fl = new ODN_SparkMax(5, MotorType.brushed);
		swerve_constants.id_mt_bl = new ODN_SparkMax(6, MotorType.brushed);
		swerve_constants.id_mt_fr = new ODN_SparkMax(7, MotorType.brushed);
        swerve_constants.id_mt_br = new ODN_SparkMax(8, MotorType.brushed);
        
		swerve_constants.id_et_fl = new ODN_CANCoder(9);
		swerve_constants.id_et_bl = new ODN_CANCoder(10);
		swerve_constants.id_et_fr = new ODN_CANCoder(11);
        swerve_constants.id_et_br = new ODN_CANCoder(12);
        
		swerve_constants.id_ed_fl = swerve_constants.id_md_fl.getEncoder();
		swerve_constants.id_ed_bl = swerve_constants.id_md_bl.getEncoder();
		swerve_constants.id_ed_fr = swerve_constants.id_md_fr.getEncoder();
        swerve_constants.id_ed_br = swerve_constants.id_md_br.getEncoder();
        
        swerve_constants.id_gyro = 13;
        
        swerve_constants.trackWidth = 18.5*2.54/100;
        swerve_constants.wheelBase = 18.5*2.54/100;

		swerve_constants.driveKinematics = new SwerveDriveKinematics(
            new Translation2d(swerve_constants.wheelBase / 2, swerve_constants.trackWidth / 2),
            new Translation2d(swerve_constants.wheelBase / 2, -swerve_constants.trackWidth / 2),
            new Translation2d(-swerve_constants.wheelBase / 2, swerve_constants.trackWidth / 2),
            new Translation2d(-swerve_constants.wheelBase / 2, -swerve_constants.trackWidth / 2));

		swerve_constants.offset_fl = -20.83;
		swerve_constants.offset_bl = 72.510;
		swerve_constants.offset_fr = 95.098;
		swerve_constants.offset_br = 48.076;
    }

    public static ElevatorSubsystem.Constants elevator_constants = new ElevatorSubsystem.Constants();
    static {
        elevator_constants.motor = new ODN_SparkMax(8, MotorType.brushed);
        elevator_constants.encoder = new ODN_CANCoder(9);
    
        elevator_constants.encoderFactor = 1;
    
        elevator_constants.maxVelocity = 2;
        elevator_constants.maxAcceleration = 2;
    
        elevator_constants.kp = 1; 
        elevator_constants.ki = 0; 
        elevator_constants.kd = 0;
    
        elevator_constants.ks = 1; 
        elevator_constants.kg = 0; 
        elevator_constants.kv = 0;
    }

    
    public static int kControllerPort = 0;
    
    public static final class AutoConstants {
        public static double kMaxSpeedMetersPerSecond = 2;
        public static double kMaxAccelerationMetersPerSecondSquared = 2;
        
        public static final double kMaxAngularSpeedRadiansPerSecond = 16*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 256*Math.PI*Math.PI;
        
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
