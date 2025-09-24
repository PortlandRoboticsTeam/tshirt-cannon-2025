package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.motors.Motor;
import frc.robot.subsystems.motors.Motor.MotorType;

public class RevolverSubsystem extends SubsystemBase {
    private static final int SLOT_COUNT = 6;
    private static final double rotationPerSlot = 1.0 / SLOT_COUNT;

    private static final double kP = 0.33;              // proportional gain - strength of response to error
    private static final double kI = 0.0;               // integral gain - strength of response to accumulated error over time
    private static final double kD = 0.025;             // derivative gain - strength of response to rate of change of error
    private static final double MAX_VELOCITY = 0.5;     // rotations/sec
    private static final double MAX_ACCELERATION = 1.0; // rotations/sec^2
    private static final double DEADBAND = 0.005;       // ~1.8° threshold (stopping)

    private static final double OUTPUT_SCALE = 0.7;
    private static final double MAX_OUTPUT = 1.0;

    private final Motor motor;
    private final Encoder encoder;
    private final ProfiledPIDController pid;

    private int currentSlot = 0;

    private boolean pidActive = true;

    public RevolverSubsystem(int motorID, int encoderID) {
        this.motor = new Motor(motorID, MotorType.SparkMax);
        this.encoder = new Encoder(encoderID);

        this.pid = new ProfiledPIDController(
                kP, kI, kD,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        pid.enableContinuousInput(0.0, 1.0);

        // Snap to nearest slot on startup
        double encoderPosition = getNormalizedRotation();
        currentSlot = (int) Math.round(encoderPosition / rotationPerSlot) % SLOT_COUNT;

        encoder.setOffsetTo(encoderPosition - getTargetPosition());
        pid.reset(encoderPosition);
    }

    public void stop() {
        pidActive = false;
    }

    @Override
    public void periodic() {
        double encoderPosition = getNormalizedRotation();
        double target = getTargetPosition();
        double error = target - encoderPosition;

        if (Math.abs(error) < DEADBAND) {
            pidActive = false;
            pid.reset(encoderPosition);
        }

        if (!pidActive) {
            motor.set(0);
            return;
        }

        // Calculate motor motion required to reach target, apply scaling and clamp
        double output = pid.calculate(encoderPosition, target) * OUTPUT_SCALE;
        output = Math.max(-MAX_OUTPUT, Math.min(MAX_OUTPUT, output));
        motor.set(output);

        // Debugging
        SmartDashboard.putNumber("Revolver Measurement", encoderPosition);
        SmartDashboard.putNumber("Revolver Target", target);
        SmartDashboard.putNumber("Revolver PID Output", output);
        SmartDashboard.putNumber("Revolver Error", error);
    }

    public void nextSlot() {
        currentSlot = (currentSlot + 1) % SLOT_COUNT;
        pidActive = true;
    }

    private double getTargetPosition() {
        return currentSlot * rotationPerSlot;
    }

    /**
     * Normalizes any encoder rotation value to a fractional rotation in [0.0, 1.0).
     * <p>
     * The revolver is a circular mechanism, so we only care about the position
     * within a single rotation. This method wraps any value, positive or negative,
     * into the [0.0, 1.0) range.
     *
     * @param encoderValue The raw rotation count from the encoder (can be negative or >1)
     * @return The normalized fractional rotation in the range [0.0, 1.0)
     */
    private double getNormalizedRotation() {
        return (encoder.getValue() % 1.0 + 1.0) % 1.0;
    }

}
