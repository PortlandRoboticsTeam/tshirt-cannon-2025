package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Encoder.EncoderType;
import frc.robot.subsystems.Motor.MotorType;

public class Joint extends SubsystemBase {
    private PIDController pid;
    private Encoder encoder;
    private Motor motor;
    private double setpoint;
    private boolean inverted;
    private double minPosition, maxPosition;
    private boolean isBounded = false;
    private boolean PIDEnabled = true;

    public Joint(int motorID, int encoderID, boolean inverted, int defaultSetpoint,
            double kP, double kI, double kD, MotorType motorType, EncoderType eType) {
        pid = new PIDController(kP, kI, kD);
        pid.enableContinuousInput(-180, 180);
        this.encoder = new Encoder(encoderID, eType);
        this.motor = new Motor(motorID, motorType);
        this.setpoint = defaultSetpoint;
        this.inverted = inverted;
    }

    // --- Setpoints & Bounds ---
    public void setSetpoint(double position) {
        if (isBounded) {
            this.setpoint = Math.min(maxPosition, Math.max(minPosition, position));
        } else {
            this.setpoint = position;
        }
    }

    public void applyBounds(double min, double max) {
        isBounded = true;
        minPosition = min;
        maxPosition = max;
        setSetpoint(getSetpoint());
    }

    public void disableBounds() {
        isBounded = false;
    }

    public boolean getIsBounded() {
        return isBounded;
    }

    public double getSetpoint() {
        return setpoint;
    }

    // --- Motor control ---
    public void setSpeed(double speedPercentage) {
        double output = inverted ? -speedPercentage : speedPercentage;
        output = Math.max(-1.0, Math.min(1.0, output));
        motor.set(output);
    }

    public void stop() {
        setSpeed(0);
    }

    // --- Encoder ---
    public double getRotations() {
        return encoder.getValue();
    }

    public double getAngleDegrees() {
        return encoder.getValue() * 360 % 360;
    }

    public double getAngleRadians() {
        return encoder.getValue() * 2 * Math.PI;
    }

    public Encoder getEncoder() {
        return encoder;
    }

    // --- PID ---
    public PIDController getController() {
        return pid;
    }

    public PIDController getPID() {
        return pid;
    }

    public boolean isPIDEnabled() {
        return PIDEnabled;
    }

    public void disablePID() {
        PIDEnabled = false;
    }

    public void enablePID() {
        PIDEnabled = true;
    }

    public void setPIDValue(double p, double i, double d) {
        pid.setPID(p, i, d);
    }

    // --- Utilities ---
    public boolean isNearSetpoint(double tolerance) {
        return Math.abs(getAngleDegrees() - getSetpoint()) < tolerance;
    }

    public Command getGoToCommand(double newSetpoint, double tolerance) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> setSetpoint(newSetpoint), this),
                new WaitUntilCommand(() -> isNearSetpoint(tolerance)));
    }

    public void initialize() {
        setSetpoint(getAngleDegrees());
    }

    public Motor getMotor() {
        return motor;
    }

    @Override
    public String toString() {
        return "Setpoint: " + setpoint +
                "\nP: " + pid.getP() +
                "\nI: " + pid.getI() +
                "\nD: " + pid.getD() +
                "\nCurrent angle: " + getAngleDegrees() +
                " at voltage " + motor.getVoltage();
    }
}
