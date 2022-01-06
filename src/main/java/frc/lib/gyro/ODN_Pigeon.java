package frc.lib.gyro;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;

public class ODN_Pigeon implements ODN_Gyro {

    private final PigeonIMU m_pigeon;

    public ODN_Pigeon(int CAN) {
        m_pigeon = new PigeonIMU(CAN);
    }

    @Override
    public Rotation2d getYaw() {
        double[] d = new double[3];
        m_pigeon.getYawPitchRoll(d);
        return Rotation2d.fromDegrees(d[0]);
    }

    @Override
    public void resetYaw() {
        m_pigeon.setYaw(0);
    }
    
}
