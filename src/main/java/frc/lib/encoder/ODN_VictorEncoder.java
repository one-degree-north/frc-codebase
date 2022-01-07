package frc.lib.encoder;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.lib.motorcontroller.ODN_VictorSPX;

public class ODN_VictorEncoder implements ODN_Encoder {

    private WPI_VictorSPX m_backend;
    private int m_sensorID;

    private double m_positionFactor = 1;
    private double m_velocityFactor = 1;

    public ODN_VictorEncoder(ODN_VictorSPX talon, int sensorID) {
        m_backend = talon.getBackend();
        m_sensorID = sensorID;
    }

    @Override
    public double getPosition() {
        return m_backend.getSelectedSensorPosition(m_sensorID) * m_positionFactor;
    }

    @Override
    public double getAbsolutePosition() {
        // TODO: Add exceptions for this case
        // Cannot get absolute position so return 0
        return 0;
    }

    @Override
    public void setPosition(double newPosition) {
        m_backend.setSelectedSensorPosition(newPosition / m_positionFactor, m_sensorID, 0);
    }

    @Override
    public double getVelocity() {
        return m_backend.getSelectedSensorVelocity(m_sensorID) * m_velocityFactor;
    }

    @Override
    public void setPositionConversionFactor(double factor) {
        this.m_positionFactor = factor;
    }

    @Override
    public void setVelocityConversionFactor(double factor) {
        this.m_velocityFactor = factor;
    }

    public WPI_VictorSPX getBackend() {
        return m_backend;
    }
    
}
