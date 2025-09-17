package frc.robot.subsystems.motors;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SparkMaxMotor implements MotorController {
    private final SparkMax motor;
    private boolean inverted = false;

    public SparkMaxMotor(int id) {
        motor = new SparkMax(id, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
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
        SparkBaseConfig config = new SparkMaxConfig();
        config.idleMode(mode == Motor.IdleMode.Brake ? IdleMode.kBrake : IdleMode.kCoast);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
}
