package frc.lib.motorcontroller;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.lib.encoder.Encoder;
import frc.lib.encoder.ODN_VictorEncoder;

public class ODN_VictorSPX implements ODN_MotorController {

    private WPI_VictorSPX m_backend;

    public ODN_VictorSPX(int CAN_ID) {
        m_backend = new WPI_VictorSPX(CAN_ID);
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
        return new ODN_VictorEncoder(this, 0);
    }

    @Override
    public WPI_VictorSPX getBackend() {
        return m_backend;
    }
    
}
