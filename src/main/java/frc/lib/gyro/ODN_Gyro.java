package frc.lib.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ODN_Gyro {
    public Rotation2d getYaw();

    public void resetYaw();
}
