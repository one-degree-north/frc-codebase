package frc.lib.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ODN_Gyro {
    public Rotation2d getYaw();
    public Rotation2d getPitch();
    public Rotation2d getRoll();

    public void resetYaw();
}
