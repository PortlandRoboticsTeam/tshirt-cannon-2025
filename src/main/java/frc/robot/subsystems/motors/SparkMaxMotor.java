package frc.robot.subsystems.motors;

import com.revrobotics.spark.SparkMax;

public class SparkMaxMotor extends SparkMax implements Motor {
    public SparkMaxMotor(int id) {
        super(id, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    }
}
