package frc.lib.encoder;

public interface Encoder {
    /**
     * Gets the position of the encoder measured in degrees
     * @return The position of the encoder
     */
    public double getPosition();

    /**
     * Gets the absolute position of the encoder measured in degrees. 
     * Returns a value from 0 to 360 degrees.
     * @return The absolute position of the encoder
     */
    public double getAbsolutePosition();

    /**
     * Sets the position of the encoder. 
     * @param newPosition The new position of the encoder in degrees
     */
    public void setPosition(double newPosition);

    /**
     * Gets the velocity of the encoder measured in degrees per second
     * @return The velocity of the encoder
     */
    public double getVelocity();
}
