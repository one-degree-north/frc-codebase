package frc.lib.encoder;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.lib.motorcontroller.ODN_TalonFX;
import frc.lib.motorcontroller.ODN_TalonSRX;
import frc.lib.motorcontroller.ODN_VictorSPX;

public class ODN_PhoenixEncoder implements ODN_Encoder {

    private BaseMotorController m_backend;
    private int m_sensorID;

    private double m_positionFactor = 1;
    private double m_velocityFactor = 1;

    public ODN_PhoenixEncoder(ODN_TalonSRX talon, int sensorID) {
        m_backend = talon.getBackend();
        m_sensorID = sensorID;
    }

    public ODN_PhoenixEncoder(ODN_TalonFX talon, int sensorID) {
        m_backend = talon.getBackend();
        m_sensorID = sensorID;
    }

    public ODN_PhoenixEncoder(ODN_VictorSPX talon, int sensorID) {
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

    public WPI_TalonSRX getBackendSRX() {
        return (WPI_TalonSRX)m_backend;
    }

    public WPI_TalonFX getBackendFX() {
        return (WPI_TalonFX)m_backend;
    }

    public WPI_VictorSPX getBackendSPX() {
        return (WPI_VictorSPX)m_backend;
    }
    
}
