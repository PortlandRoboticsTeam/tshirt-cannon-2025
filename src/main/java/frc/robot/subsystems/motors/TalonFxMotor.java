package frc.robot.subsystems.motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonFxMotor implements MotorController {
    private final TalonFX motor;
    private boolean inverted = false;

    public TalonFxMotor(int id) {
        motor = new TalonFX(id);
    }

    @Override
    public void set(double output) {
        motor.set(inverted ? -output : output);
    }

    @Override
    public double getVoltage() {
        return motor.get();
    }

    @Override
    public void invert() {
        inverted = !inverted;
    }

    @Override
    public void setNeutralMode(Motor.IdleMode mode) {
        motor.setNeutralMode(mode == Motor.IdleMode.Brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public int getDeviceId() { return motor.getDeviceID(); }
}
