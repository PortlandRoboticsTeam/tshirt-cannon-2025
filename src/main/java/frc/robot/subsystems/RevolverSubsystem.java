package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.motors.Motor;
import frc.robot.subsystems.motors.MotorPool;
import frc.robot.subsystems.motors.MotorPool.MotorType;

public class RevolverSubsystem extends SubsystemBase {
    private static final int SLOT_COUNT = 6;
    private static final double rotationPerSlot = 1.0 / SLOT_COUNT;
    private static final double DEADBAND = 0.005;
    private static final double MIN_FEEDFORWARD = 0.2;
    private static final double MAX_OUTPUT = 1.0;

    private final Motor motor = MotorPool.create(RobotContainer.REVOLVER_MOTOR_ID, MotorType.SparkMax);
    private final Encoder encoder = new Encoder(RobotContainer.REVOLVER_ENCODER_ID);

    private int currentSlot = 0;
    private boolean motorActive = false;

    /**
     * Snap to nearest slot on startup
     */
    public RevolverSubsystem() {
        double encoderPosition = encoder.getNormalizedRotation();
        currentSlot = (int) Math.round(encoderPosition / rotationPerSlot) % SLOT_COUNT;
    }

    @Override
    public void periodic() {
        // read current and target position
        double encoderPosition = encoder.getNormalizedRotation();
        double target = getTargetPosition();

        // calculate shortest angular error (-0.5 to 0.5)
        double error = target - encoderPosition;
        error = (error + 0.5) % 1.0 - 0.5;  

        // calculate output with min to overcome static friction and clamp for saftey
        double output = Math.copySign(MIN_FEEDFORWARD, error);
        output = Math.max(-MAX_OUTPUT, Math.min(MAX_OUTPUT, output));

        if (Math.abs(error) < DEADBAND) {
            motorActive = false;
        }

        // log all calculations for debugging/tuning
        SmartDashboard.putNumber("Revolver Position", encoderPosition);
        SmartDashboard.putNumber("Revolver Target", target);
        SmartDashboard.putNumber("Revolver Error", error);
        SmartDashboard.putNumber("Revolver Output", output);
        SmartDashboard.putBoolean("Revolver Active", motorActive);

        // when active, run motor to correct position; otherwise stop
        motor.set(motorActive ? output * -1 : 0);
    }

    public Command nextSlot() {
        return runOnce(() -> {
            currentSlot = (currentSlot + 1) % SLOT_COUNT;
            motorActive = true;
        });
    }

    public Command stop() {
        return runOnce(() -> motorActive = false);
    }

    private double getTargetPosition() {
        return currentSlot * rotationPerSlot;
    }

}
