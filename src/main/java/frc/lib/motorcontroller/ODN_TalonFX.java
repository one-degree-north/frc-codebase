package frc.lib.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.lib.encoder.ODN_Encoder;
import frc.lib.encoder.ODN_PhoenixEncoder;

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
    public ODN_Encoder getEncoder() {
        ODN_Encoder e = new ODN_PhoenixEncoder(this, 0);
        e.setPositionConversionFactor(60.0/2048*10);
        e.setVelocityConversionFactor(60.0/2048*10);
        return e;
    }

    @Override
    public WPI_TalonFX getBackend() {
        return m_backend;
    }

    public static final double MAX_SPEED = 6380;

    @Override
    public void setRealSpeed(double speed) {
        m_backend.setVoltage(12*speed/MAX_SPEED);
    }
    
}
