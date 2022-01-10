package frc.lib.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.lib.encoder.ODN_Encoder;
import frc.lib.encoder.ODN_PhoenixEncoder;

public class ODN_TalonSRX implements ODN_MotorController {

    private WPI_TalonSRX m_backend;

    public ODN_TalonSRX(int CAN_ID) {
        m_backend = new WPI_TalonSRX(CAN_ID);
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
    public ODN_Encoder getEncoder() {
        return new ODN_PhoenixEncoder(this, 0);
    }

    @Override
    public WPI_TalonSRX getBackend() {
        return m_backend;
    }

    @Override
    public void setRealSpeed(double speed) {
        m_backend.set(ControlMode.Velocity, speed);
    }
    
}
