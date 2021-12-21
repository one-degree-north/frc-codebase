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


    public static SwerveDriveSubsystem.Constants swerveDriveSubsystemConstants = new SwerveDriveSubsystem.Constants();
    static {
        swerveDriveSubsystemConstants.wheelDiameterMeters = 4/39.37;
        swerveDriveSubsystemConstants.maxSpeedMetersPerSecond = 3.38;
        swerveDriveSubsystemConstants.gyroReversed = false;

        swerveDriveSubsystemConstants.id_mt_fl = new ODN_SparkMax(1, MotorType.brushless);
        swerveDriveSubsystemConstants.id_mt_bl = new ODN_SparkMax(3, MotorType.brushless);
        swerveDriveSubsystemConstants.id_mt_fr = new ODN_SparkMax(5, MotorType.brushless);
        swerveDriveSubsystemConstants.id_mt_br = new ODN_SparkMax(7, MotorType.brushless);
        
        swerveDriveSubsystemConstants.id_md_fl = new ODN_SparkMax(2, MotorType.brushless);
        swerveDriveSubsystemConstants.id_md_bl = new ODN_SparkMax(4, MotorType.brushless);
        swerveDriveSubsystemConstants.id_md_fr = new ODN_SparkMax(6, MotorType.brushless);        
        swerveDriveSubsystemConstants.id_md_br = new ODN_SparkMax(8, MotorType.brushless);

        swerveDriveSubsystemConstants.id_ed_fr = swerveDriveSubsystemConstants.id_md_fr.getEncoder();
        swerveDriveSubsystemConstants.id_ed_fl = swerveDriveSubsystemConstants.id_md_fl.getEncoder();
        swerveDriveSubsystemConstants.id_ed_br = swerveDriveSubsystemConstants.id_md_br.getEncoder();        
        swerveDriveSubsystemConstants.id_ed_bl = swerveDriveSubsystemConstants.id_md_bl.getEncoder();

        swerveDriveSubsystemConstants.id_et_fr = new ODN_CANCoder(11);     
        swerveDriveSubsystemConstants.id_et_fl = new ODN_CANCoder(12);
        swerveDriveSubsystemConstants.id_et_br = new ODN_CANCoder(10);
        swerveDriveSubsystemConstants.id_et_bl = new ODN_CANCoder(9);

        swerveDriveSubsystemConstants.driveKinematics = new SwerveDriveKinematics(new Translation2d[]{
            new Translation2d(0.235,0.235),
            new Translation2d(0.235,-0.235),
            new Translation2d(-0.235,0.235),
            new Translation2d(-0.235,-0.235)
        });
        
        swerveDriveSubsystemConstants.offset_fl = 7.471;
        swerveDriveSubsystemConstants.offset_bl = 251.542-180;
        swerveDriveSubsystemConstants.id_md_bl.setInverted(true);
        swerveDriveSubsystemConstants.offset_fr = 316.055-180;
        swerveDriveSubsystemConstants.id_md_fr.setInverted(true);
        swerveDriveSubsystemConstants.offset_br = 338.467-360;

        swerveDriveSubsystemConstants.wheelBase = 0.47;
        swerveDriveSubsystemConstants.trackWidth = 0.47;

        
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
