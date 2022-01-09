package frc.lib.sensor;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class ODN_ColorSensor {

    private static final I2C.Port i2cPort = I2C.Port.kOnboard;

    private ColorSensorV3 m_sensor = new ColorSensorV3(i2cPort);

    public ODN_ColorSensor(I2C.Port port) {
        m_sensor = new ColorSensorV3(port);
    }

    public ODN_ColorSensor() {
        this(I2C.Port.kOnboard);
    }

    public Color getColor() {
        return m_sensor.getColor();
    }

    public static ColorMatch createMatcher(double confidence, Color... colors) {
        ColorMatch match = new ColorMatch();
        match.setConfidenceThreshold(confidence);
        for(Color c: colors)
            match.addColorMatch(c);
        return match;
    }

    public Color match(ColorMatch matcher) {
        Color detectedColor = m_sensor.getColor();
        ColorMatchResult match = matcher.matchColor(detectedColor);
        return match.color;
    }
}
