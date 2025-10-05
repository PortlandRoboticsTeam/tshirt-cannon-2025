package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

public class Encoder {
    private final CANcoder cancoder;
    private double offset = 0;

    public Encoder(int id) {
        cancoder = new CANcoder(id);
    }

    public double getValue() {
        return cancoder.getPosition().getValueAsDouble() - offset;
    }

    /**
     * Normalizes any encoder rotation value to a fractional rotation in [0.0, 1.0).
     * <p>
     * The revolver is a circular mechanism, so we only care about the position
     * within a single rotation. This method wraps any value, positive or negative,
     * into the [0.0, 1.0) range.
     *
     * @param encoderValue The raw rotation count from the encoder (can be negative or >1)
     * @return The normalized fractional rotation in the range [0.0, 1.0)
     */
    public double getNormalizedRotation() {
        return (getValue() % 1.0 + 1.0) % 1.0;
    }
}
