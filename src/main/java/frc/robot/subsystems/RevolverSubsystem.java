package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.motors.Motor;
import frc.robot.subsystems.motors.Motor.MotorType;

public class RevolverSubsystem extends SubsystemBase {
    private final Motor motor;
    private final Encoder encoder;
    private final PIDController pid;

    private static final int slotCount = 6;
    private int currentSlot = 0;

    private final double slotRotations; // rotations per slot
    private static final double tolerance = 0.01; // rotations

    private boolean pidActive = true; // controls whether PID runs
    private static final double deadband = 0.005; // stop threshold (~0.5% of rotation)

    public RevolverSubsystem(int motorID, int encoderID) {
        this.motor = new Motor(motorID, MotorType.SparkMax);
        this.encoder = new Encoder(encoderID);

        // PID gains tuned for smooth motion
        this.pid = new PIDController(0.33, 0.0, 0.025);
        pid.enableContinuousInput(0.0, 1.0); // circular mechanism

        this.slotRotations = 1.0 / slotCount;

        // Zero revolver at current position
        encoder.setOffsetTo(encoder.getValue() - getTargetPosition());
    }

    // --- Slot control ---
    public void nextSlot() {
        currentSlot = (currentSlot + 1) % slotCount;
        pidActive = true; // re-enable PID for new target
    }

    public double getTargetPosition() {
        return currentSlot * slotRotations;
    }

    public boolean atTarget() {
        return Math.abs(encoder.getValue() - getTargetPosition()) < tolerance;
    }

    // --- Periodic control ---
    @Override
    public void periodic() {
        double measurement = (encoder.getValue() % 1.0 + 1.0) % 1.0;
        double target = getTargetPosition();
        double error = target - measurement;

        if (!pidActive) {
            motor.set(0);
            return;
        }

        // Deadband: stop motor if close enough
        if (Math.abs(error) < deadband) {
            motor.set(0);
            pidActive = false; // freeze PID
            return;
        }

        // PID calculation
        double output = pid.calculate(measurement, target);

        // Apply 70% scaling
        output *= 0.7;

        // Clamp to motor limits
        output = Math.max(-1.0, Math.min(1.0, output));
        motor.set(output);

        // SmartDashboard for debugging
        SmartDashboard.putNumber("Revolver Measurement", measurement);
        SmartDashboard.putNumber("Revolver Target", target);
        SmartDashboard.putNumber("Revolver PID Output", output);
        SmartDashboard.putNumber("Revolver Error", error);
    }

    // --- Stop the revolver immediately ---
    public void stop() {
        pidActive = false;
        motor.set(0);
    }

    // --- Command to move to next slot ---
    public Command goToNextSlotCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(this::nextSlot, this),
            new WaitUntilCommand(this::atTarget)
        );
    }
}
