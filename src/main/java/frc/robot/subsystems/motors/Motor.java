package frc.robot.subsystems.motors;

public class Motor {
    public enum MotorType { SparkMax, Talon }
    public enum IdleMode { Brake, Coast }

    private final MotorController controller;

    public Motor(int id, MotorType type) {
        switch(type) {
            case SparkMax -> controller = new SparkMaxMotor(id);
            case Talon -> controller = new TalonFxMotor(id);
            default -> throw new IllegalArgumentException("Unknown motor type");
        }
    }

    public void set(double output) { controller.set(output); }
    public double getVoltage() { return controller.getVoltage(); }
    public void invert() { controller.invert(); }
    public void setNeutralMode(IdleMode mode) { controller.setNeutralMode(mode); }
    public int getDeviceId() { return controller.getDeviceId();}
}
