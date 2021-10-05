package frc.lib.encoder;

import com.ctre.phoenix.sensors.CANCoder;

public class ODN_CANCoder implements Encoder {

    private CANCoder m_backend;

    public ODN_CANCoder(int CAN_ID) {
        m_backend = new CANCoder(CAN_ID);
    }

    @Override
    public double getPosition() {
        return m_backend.getPosition();
    }

    @Override
    public double getAbsolutePosition() {
        return m_backend.getAbsolutePosition();
    }

    @Override
    public void setPosition(double newPosition) {
        m_backend.setPosition(newPosition);
    }

    @Override
    public double getVelocity() {
        return m_backend.getVelocity();
    }

    public CANCoder getBackend() {
        return m_backend;
    }
    
}
