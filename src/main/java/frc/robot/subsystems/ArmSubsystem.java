package frc.robot.subsystems;

import java.util.stream.Stream;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.motors.Motor;
import frc.robot.subsystems.motors.MotorPool;
import frc.robot.subsystems.motors.MotorPool.MotorType;

public class ArmSubsystem extends SubsystemBase {
    private static final double MAX_MOTOR_OUTPUT = 1.0;
    private static final int MOTOR_FORWARD = 1;
    private static final int MOTOR_REVERSE = -1;

    private static final double MIN_SHOULDER_POSITION = 0.1;
    private static final double MAX_SHOULDER_POSITION = 0.6;
    private static final double MIN_ELBOW_POSITION = 0.5;
    private static final double MAX_ELBOW_POSITION = 0.6;

    private final ArmJoint shoulder = new ArmJoint(
        "Shoulder",
        MotorPool.create(RobotContainer.SHOULDER_MOTOR_ID, MotorType.SparkMax),
        new Encoder(RobotContainer.SHOULDER_ENCODER_ID),
        new PIDController(0.8, 0.0, 0.05),
        MOTOR_REVERSE,
        MIN_SHOULDER_POSITION, 
        MAX_SHOULDER_POSITION);

    private final ArmJoint elbow = new ArmJoint(
        "Elbow",
        MotorPool.create(RobotContainer.ELBOW_MOTOR_ID, MotorType.SparkMax),
        new Encoder(RobotContainer.ELBOW_ENCODER_ID),
        new PIDController(0.8, 0.0, 0.05),
        MOTOR_FORWARD,
        MIN_ELBOW_POSITION, 
        MAX_ELBOW_POSITION);

    @Override
    public void periodic() {
        Stream.of(shoulder, elbow).forEach(joint -> {
            // read current and target position
            double encoderPosition = joint.encoder.getNormalizedRotation();
            double target = Math.max(joint.minPosition, Math.min(joint.maxPosition, joint.pid.getSetpoint()));

            // calculate output with min to overcome static friction and clamp for saftey
            double output = joint.pid.calculate(encoderPosition, target);
            output = Math.max(-MAX_MOTOR_OUTPUT, Math.min(MAX_MOTOR_OUTPUT, output));
            double error = joint.pid.getError();

            if (Math.abs(error) < 0.01) {
                joint.motorActive = false;
            }

            // log all calculations for debugging/tuning
            SmartDashboard.putNumber(joint.name + " Position", encoderPosition);
            SmartDashboard.putNumber(joint.name + " Target", target);
            SmartDashboard.putNumber(joint.name + " Error", error);
            SmartDashboard.putNumber(joint.name + " Output", output);
            SmartDashboard.putBoolean(joint.name + " Active", joint.motorActive);

            double motorPower = joint.motorActive ? output * joint.motorDirection : 0;
            SmartDashboard.putNumber(joint.name + " Motor Power", motorPower);

            // when active, run motor to correct position; otherwise stop
            joint.motor.set(motorPower);
        });    
    }

    public Command extend() {
        return runOnce(() -> {
            shoulder.pid.setSetpoint(MAX_SHOULDER_POSITION);
            elbow.pid.setSetpoint(MAX_ELBOW_POSITION);
            shoulder.motorActive = true;
            elbow.motorActive = true;
        });
    }

    public Command retract() {
        return runOnce(() -> {
            shoulder.pid.setSetpoint(MIN_SHOULDER_POSITION);
            elbow.pid.setSetpoint(MIN_ELBOW_POSITION);
            shoulder.motorActive = true;
            elbow.motorActive = true;
        });
    }

    public Command stop() {
        return runOnce(() -> {
            shoulder.motorActive = false;
            elbow.motorActive = false;
        });
    }

    private class ArmJoint {
        final String name;
        final Motor motor;
        final Encoder encoder;
        final PIDController pid;
        final int motorDirection;
        final double minPosition;
        final double maxPosition;

        boolean motorActive = false;

        ArmJoint(String n, Motor m, Encoder e, PIDController p, int md, double min, double max) {
            this.name = n;
            this.motor = m;
            this.encoder = e;
            this.pid = p;
            this.motorDirection = md;
            this.minPosition = min;
            this.maxPosition = max;
        }
    }
}
