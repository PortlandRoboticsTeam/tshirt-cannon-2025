package frc.robot.subsystems.motors;

public interface MotorController {
    void set(double output);
    double getVoltage();
    void invert();
    void setNeutralMode(Motor.IdleMode mode);
    int getDeviceId();
}
