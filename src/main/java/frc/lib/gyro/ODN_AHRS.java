package frc.lib.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;

public class ODN_AHRS implements ODN_Gyro {

    private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

    @Override
    public Rotation2d getYaw() {
        /*
        if (m_navx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(180 - m_navx.getFusedHeading());
        }*/
        return Rotation2d.fromDegrees(m_navx.getYaw());
    }

    @Override
    public void resetYaw() {
        m_navx.zeroYaw();
    }
    
}
