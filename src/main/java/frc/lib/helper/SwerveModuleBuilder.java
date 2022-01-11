package frc.lib.helper;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.SwerveModuleFactory;
import com.swervedrivespecialties.swervelib.ctre.CanCoderAbsoluteConfiguration;
import com.swervedrivespecialties.swervelib.ctre.CanCoderFactoryBuilder;
import com.swervedrivespecialties.swervelib.ctre.Falcon500DriveControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.ctre.Falcon500SteerConfiguration;
import com.swervedrivespecialties.swervelib.ctre.Falcon500SteerControllerFactoryBuilder;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveModuleBuilder {
    public static final ModuleConfiguration NEO_MK3_STANDARD_CONFIG = Mk3SwerveModuleHelper.GearRatio.STANDARD.getConfiguration();
    public static final ModuleConfiguration Falcon_WCP_X_STANDARD_CONFIG = new ModuleConfiguration(
        0.1016,
        (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0),
        true,
        (15.0 / 32.0) * (10.0 / 60.0),
        true);

    public static SwerveModule createNEO_MK3_STANDARD(String name, int posX, int posY, int drive_motor, int steer_motor, int steer_encoder,
    double steer_offset) {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        return Mk3SwerveModuleHelper.createNeo(
                tab.getLayout(name, BuiltInLayouts.kList).withSize(2, 4).withPosition(posX, posY),
                Mk3SwerveModuleHelper.GearRatio.STANDARD, drive_motor,
                steer_motor, steer_encoder,
                steer_offset);
    }

    public static SwerveModule createFalcon_WCP_X_STANDARD(String name, int posX, int posY, int drive_motor, int steer_motor,
            int steer_encoder, double steer_offset) {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        return new SwerveModuleFactory<>(
            Falcon_WCP_X_STANDARD_CONFIG,
                new Falcon500DriveControllerFactoryBuilder()
                        .withVoltageCompensation(Double.NaN)
                        .withCurrentLimit(Double.NaN)
                        .build(),
                new Falcon500SteerControllerFactoryBuilder()
                        .withVoltageCompensation(Double.NaN)
                        .withPidConstants(0.2, 0.0, 0.1)
                        .withCurrentLimit(Double.NaN)
                        .build(new CanCoderFactoryBuilder()
                                .withReadingUpdatePeriod(100)
                                .build())).create(
                                    tab.getLayout(name, BuiltInLayouts.kList).withSize(2, 4).withPosition(posX, posY),
                                    drive_motor,
                                        new Falcon500SteerConfiguration<>(
                                            steer_motor,
                                                new CanCoderAbsoluteConfiguration(steer_encoder, steer_offset)));
    }
}
