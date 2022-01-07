package frc.lib.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public class ODN_NullGyro implements ODN_Gyro {

    @Override
    public Rotation2d getYaw() {
        return new Rotation2d();
    }

    @Override
    public void resetYaw() {
    }
    
}