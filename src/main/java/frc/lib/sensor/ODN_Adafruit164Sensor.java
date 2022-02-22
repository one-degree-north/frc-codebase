package frc.lib.sensor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class ODN_Adafruit164Sensor implements ODN_DistanceSensor {

    private final AnalogInput m_input;

    public ODN_Adafruit164Sensor(int pin) {
        m_input = new AnalogInput(pin);
    }

    @Override
    public double getDistanceInches() {
        double avg = 0;
        for(int i=0;i<5;i++) {
            double voltage_scale_factor = 5/RobotController.getVoltage5V();

            double value = m_input.getVoltage() * voltage_scale_factor;
            //System.out.println(value);
            double currentDistanceInches = (168 * (1/value) - 4) / 13;

            avg += currentDistanceInches;
        }
        

        return avg/5;
    }
}