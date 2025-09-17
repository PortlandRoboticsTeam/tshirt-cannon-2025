package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Encoder.EncoderType;
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

    public RevolverSubsystem(int motorID, int encoderID) {
        this.motor = new Motor(motorID, MotorType.SparkMax);
        this.encoder = new Encoder(encoderID, EncoderType.CANCoder);
        this.pid = new PIDController(0.02, 0.0, 0.0001); // tune PID

        this.slotRotations = 1.0 / slotCount;
        encoder.setOffsetTo(0);
    }

    // --- Slot control ---
    public void nextSlot() {
        currentSlot = (currentSlot + 1) % slotCount;
    }

    private double getTargetPosition() {
        return currentSlot * slotRotations;
    }

    public boolean atTarget() {
        return Math.abs(encoder.getValue() - getTargetPosition()) < tolerance;
    }

    // --- PID control ---
    @Override
    public void periodic() {
        double measurement = encoder.getValue();
        double output = pid.calculate(measurement, getTargetPosition());

        // clamp output
        output = Math.max(-1.0, Math.min(1.0, output));
        motor.set(output);
    }

    public void stop() {
        motor.set(0);
    }

    public Command goToNextSlotCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(this::nextSlot, this),
            new WaitUntilCommand(this::atTarget)
        );
    }

}
