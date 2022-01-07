package frc.lib.encoder;

public class ODN_NullEncoder implements ODN_Encoder {

    @Override
    public double getPosition() {
        return 0;
    }

    @Override
    public double getAbsolutePosition() {
        return 0;
    }

    @Override
    public void setPosition(double newPosition) {
    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public void setPositionConversionFactor(double factor) {
    }

    @Override
    public void setVelocityConversionFactor(double factor) {
    }
    
}
