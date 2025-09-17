package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

public class Encoder {
    private final CANcoder cancoder;
    private double offset = 0;

    public Encoder(int id) {
        cancoder = new CANcoder(id);
    }

    /** Returns raw rotations */
    public double getValue() {
        return cancoder.getPosition().getValueAsDouble() - offset;
    }

    /** Returns angle in degrees (0–360) */
    public double getAngleDegrees() {
        return getValue() * 360 % 360;
    }

    /** Set an offset so zero is at a specific position */
    public void setOffset(double offset) {
        this.offset = offset;
    }

    /** Set zero to a specific position */
    public void setOffsetTo(double newOffset) {
        offset = getValue() - newOffset;
    }

    /** Returns true if encoder is connected */
    public boolean isConnected() {
        return cancoder.isConnected();
    }
}
