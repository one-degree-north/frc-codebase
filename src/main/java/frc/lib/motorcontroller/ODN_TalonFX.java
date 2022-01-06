package frc.lib.motorcontroller;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.lib.encoder.Encoder;
import frc.lib.encoder.ODN_TalonEncoder;

public class ODN_TalonFX implements ODN_MotorController {

    private WPI_TalonFX m_backend;

    public ODN_TalonFX(int CAN_ID) {
        m_backend = new WPI_TalonFX(CAN_ID);
    }

    @Override
    public void set(double speed) {
        m_backend.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        m_backend.setVoltage(voltage);
    }

    @Override
    public void setInverted(boolean isInverted) {
        m_backend.setInverted(isInverted);
    }

    @Override
    public Encoder getEncoder() {
        return new ODN_TalonEncoder(this, 0);
    }

    @Override
    public WPI_TalonFX getBackend() {
        return m_backend;
    }
    
}
