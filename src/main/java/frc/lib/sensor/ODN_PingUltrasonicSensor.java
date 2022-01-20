package frc.lib.sensor;

import edu.wpi.first.wpilibj.Ultrasonic;

public class ODN_PingUltrasonicSensor implements ODN_DistanceSensor {

    private final Ultrasonic m_ultrasonic;

    public ODN_PingUltrasonicSensor(int pin1, int pin2) {
        m_ultrasonic = new Ultrasonic(pin1, pin2);
    }

    @Override
    public double getDistanceInches() {
        return m_ultrasonic.getRangeInches();
    }
}
