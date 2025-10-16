package frc.robot.subsystems.motors;

public class MotorPool {
    public enum MotorType { SparkMax, Talon }

    public static Motor create(int id, MotorType type) {
        switch(type) {
            case SparkMax: 
                return new SparkMaxMotor(id);
            case Talon:
                return new TalonFxMotor(id);
            default: 
                throw new IllegalArgumentException("Unknown motor type");
        }
    }
}
