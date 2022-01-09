package frc.lib.sensor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class ODN_AnalogUltrasonicSensor implements ODN_UltrasonicSensor {

    private final AnalogInput m_input;

    public ODN_AnalogUltrasonicSensor(int pin) {
        m_input = new AnalogInput(pin);
    }

    @Override
    public double getDistanceInches() {
        double raw_value = m_input.getValue();

        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        
        double currentDistanceInches = raw_value * voltage_scale_factor * 0.0492;

        return currentDistanceInches;
    }
}