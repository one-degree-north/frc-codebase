package frc.lib.encoder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.lib.motorcontroller.ODN_TalonSRX;

public class ODN_TalonEncoder implements Encoder {

    private WPI_TalonSRX m_backend;
    private int m_sensorID;

    public ODN_TalonEncoder(ODN_TalonSRX talon, int sensorID) {
        m_backend = talon.getBackend();
        m_sensorID = sensorID;
    }

    @Override
    public double getPosition() {
        return m_backend.getSelectedSensorPosition(m_sensorID);
    }

    @Override
    public double getAbsolutePosition() {
        // TODO: Add exceptions for this case
        // Cannot get absolute position so return 0
        return 0;
    }

    @Override
    public void setPosition(double newPosition) {
        m_backend.setSelectedSensorPosition(newPosition, m_sensorID, 0);
    }

    @Override
    public double getVelocity() {
        return m_backend.getSelectedSensorVelocity(m_sensorID);
    }

    public WPI_TalonSRX getBackend() {
        return m_backend;
    }
    
}
