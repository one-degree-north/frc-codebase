package frc.lib.encoder;

import com.revrobotics.RelativeEncoder;

public class ODN_CANEncoder implements ODN_Encoder {

    private RelativeEncoder m_backend;

    public ODN_CANEncoder(RelativeEncoder backend) {
        m_backend = backend;
    }

    @Override
    public double getPosition() {
        return m_backend.getPosition();
    }

    @Override
    public double getAbsolutePosition() {
        // TODO: Add exceptions for this case
        // Cannot get absolute position so return 0
        return 0;
    }

    @Override
    public void setPosition(double newPosition) {
        m_backend.setPosition(newPosition);
    }

    @Override
    public double getVelocity() {
        return m_backend.getVelocity();
    }

    @Override
    public void setPositionConversionFactor(double factor) {
        m_backend.setPositionConversionFactor(factor);
    }

    @Override
    public void setVelocityConversionFactor(double factor) {
        m_backend.setVelocityConversionFactor(factor);
    }

    public RelativeEncoder getBackend() {
        return m_backend;
    }
    
}
