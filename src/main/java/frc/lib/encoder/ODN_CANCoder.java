package frc.lib.encoder;

import com.ctre.phoenix.sensors.CANCoder;

public class ODN_CANCoder implements ODN_Encoder {

    private CANCoder m_backend;

    private double m_positionFactor = 1;
    private double m_velocityFactor = 1;

    public ODN_CANCoder(int CAN_ID) {
        m_backend = new CANCoder(CAN_ID);
    }

    @Override
    public double getPosition() {
        return m_backend.getPosition()*m_positionFactor;
    }

    @Override
    public double getAbsolutePosition() {
        return m_backend.getAbsolutePosition();
    }

    @Override
    public void setPosition(double newPosition) {
        m_backend.setPosition(newPosition / m_positionFactor);
    }

    @Override
    public double getVelocity() {
        return m_backend.getVelocity()*m_velocityFactor;
    }

    @Override
    public void setPositionConversionFactor(double factor) {
        this.m_positionFactor = factor;
    }

    @Override
    public void setVelocityConversionFactor(double factor) {
        this.m_velocityFactor = factor;
    }

    public CANCoder getBackend() {
        return m_backend;
    }
    
}
