package frc.robot.subsystems.motors;

import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFxMotor extends TalonFX implements Motor {
    public TalonFxMotor(int id) {
        super(id);
    }
}
