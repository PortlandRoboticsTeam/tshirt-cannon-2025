package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Encoder {
    CANcoder cancoder;
    DutyCycleEncoder dutyEncoder;
    EncoderType type;
    double offset = 0;

    public enum EncoderType {
        CANCoder,
        DutyCycle,
    }

    public Encoder(int id, EncoderType type) {
        this.type = type;
        switch (type) {
            case CANCoder:
                cancoder = new CANcoder(id);
                break;
            case DutyCycle:
                dutyEncoder = new DutyCycleEncoder(id);
            default:
                break;
        }
    }

    public double getValue() {
        switch (type) {
            case CANCoder:
                return cancoder.getPosition().getValueAsDouble() - offset;
            case DutyCycle:
                return dutyEncoder.get() - offset;
            default:
                return -1;
        }
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void setOffsetTo(double newOffset) {
        setOffset(0);
        setOffset(getValue() - newOffset);
    }

    public boolean isConnected() {
        switch (type) {
            case CANCoder:
                return cancoder.isConnected();
            case DutyCycle:
                return dutyEncoder.isConnected();
            default:
                return false;
        }
    }
}