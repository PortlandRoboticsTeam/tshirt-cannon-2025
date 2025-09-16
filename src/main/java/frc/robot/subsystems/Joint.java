package frc.robot.subsystems;

// Import necessary libraries for hardware control and PID management
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Encoder.EncoderType;
import frc.robot.subsystems.Motor.MotorType;

// Joint class extends SubsystemBase, allowing integration into the FRC command-based framework
public class Joint extends SubsystemBase {
    private PIDController pid; // PID controller for managing motor position
    private Encoder encoder; // Encoder to read joint position
    private Motor motor; // Motor controlling the joint movement
    private double setpoint; // the desired value for the PID contoller
    private int jointNum; // ID of the joint
    private boolean inverted; // Weather or not the encoder is inverted
    private double minPosition, maxPosition;
    private boolean isBounded = false;
    private boolean PIDEnabled = true;

    // Constructor to initialize the Joint subsystem
    public Joint(int jointNum, int motorID, int encoderID, boolean inverted, int defaultSetpoint, double kP, double kI,
            double kD, MotorType motorType, EncoderType eType) {
        // Initialize PID controller with specified gains
        pid = new PIDController(kP, kI, kD);
        // Enable continuous input for angles, wrapping around at -180 to 180 degrees
        pid.enableContinuousInput(-180, 180);
        // Initialize encoder with the given encoder ID
        encoder = new Encoder(encoderID, eType);
        // Initialize motor with the given motor ID
        motor = new Motor(motorID, motorType);
        // sets the default setpoint
        this.setpoint = defaultSetpoint;
        // Initalize the jointNum
        this.jointNum = jointNum;
        // Initalized inverted
        this.inverted = inverted;
    }

    public Joint(int jointNum, int motorID, Encoder encoder, boolean inverted, int defaultSetpoint, double kP,
            double kI, double kD, MotorType motorType) {
        // Initialize PID controller with specified gains
        pid = new PIDController(kP, kI, kD);
        // Enable continuous input for angles, wrapping around at -180 to 180 degrees
        pid.enableContinuousInput(-180, 180);
        // Initialize encoder with the given encoder ID
        this.encoder = encoder;
        // Initialize motor with the given motor ID
        motor = new Motor(motorID, motorType);
        // sets the default setpoint
        this.setpoint = defaultSetpoint;
        // Initalize the jointNum
        this.jointNum = jointNum;
        // Initalized inverted
        this.inverted = inverted;
    }

    // Periodic method called every scheduler loop; can be used for periodic tasks
    @Override
    public void periodic() {
        // Currently empty; can be used to update state or perform checks
        SmartDashboard.putNumber("current angle for joint number " + jointNum, getAngleDegrees());
        SmartDashboard.putNumber("current setPoint Degrees for joint number " + jointNum, setpoint);
        SmartDashboard.putString("joint " + jointNum + " info", toString());
        SmartDashboard.putBoolean("is near pos", isNearSetpoint(5));
    }

    // Method to set the current position index
    public void setSetpoint(double position) {
        if (isBounded) {
            this.setpoint = Math.min(maxPosition, Math.max(minPosition, position));
        } else {
            this.setpoint = position;
        }
    }

    // set bounds
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

    // Method to get the current position index
    public double getSetpoint() {
        return setpoint;
    }

    // Method to set the motor speed as a percentage (from -1.0 to 1.0)
    public void setSpeed(double speedPercentage) {
        double output;
        if (inverted) {
            output = Math.max(-1.0, Math.min(1.0, -speedPercentage));
            SmartDashboard.putNumber("Joint " + getID() + " output", output);
        } else {
            output = Math.max(-1.0, Math.min(1.0, speedPercentage));
            SmartDashboard.putNumber("Joint " + getID() + " output", output);
        }
        motor.set(output);
    }

    // Method to get the encoder's position in terms of rotations
    public double getRotations() {
        return encoder.getValue();
    }

    // Method to get the angle in degrees based on encoder position
    public double getAngleDegrees() {
        return encoder.getValue() * 360 % 360;
    }

    // Method to get the angle in radians based on encoder position
    public double getAngleRadians() {
        return encoder.getValue() * 2 * Math.PI;
    }

    // Method to get the PID controller
    public PIDController getController() {
        return pid;
    }

    // Method to get the PID controller
    public int getID() {
        return jointNum;
    }

    // Method to stop motor
    public void stop() {
        setSpeed(0);
    }

    // get the PID controler
    public PIDController getPID() {
        return pid;
    }

    public void smoke() {
        SmartDashboard.putString("Smoke", "Dont do Drugs");
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

    @Override
    public String toString() {
        String start = "Joint number:" + jointNum + "\n setpoint : " + setpoint + "\n";
        String pidString = "P: " + pid.getP() + "\nI: " + pid.getI() + "\nD: " + pid.getD() + "\n";
        String currentPos = "current angle:" + getAngleDegrees() + " at voltage " + motor.getVoltage();
        return start + pidString + currentPos;
    }

    public Encoder getEncoder() {
        return encoder;
    }

    public boolean isNearSetpoint(double d) {
        return Math.abs(getAngleDegrees() - getSetpoint()) < d;
    }

    public Command getGoToCommand(double newSetpoint, double tolerance) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> setSetpoint(newSetpoint), this),
                new WaitUntilCommand(() -> isNearSetpoint(tolerance)));
    }

    public void setPIDValue(double p, double i, double d) {
        pid.setPID(p, i, d);
    }

    public void initialize() {
        setSetpoint(getAngleDegrees());
    }

    public Motor getMotor() {
        return motor;
    }

}
