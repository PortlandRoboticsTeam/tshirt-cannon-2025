package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    private static final double MAXIMUM_SPEED = Units.feetToMeters(4);
    private static final TelemetryVerbosity TELEMETRY_VERBOSITY = TelemetryVerbosity.LOW;

    private final SwerveDrive swerveDrive;

    public SwerveSubsystem() {
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve/neo");
            SwerveDriveTelemetry.verbosity = TELEMETRY_VERBOSITY;
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(MAXIMUM_SPEED);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load swerve config", e);
        }
    }

    /**
     * Command to drive the robot in robot-relative mode.
     *
     * @param translationX     Forward/backward input
     * @param translationY     Left/right input
     * @param angularRotationX Rotation input
     * @param speedController  Speed scaling input
     * @return Drive command
     */
    public Command driveCommand(DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier angularRotationX,
            DoubleSupplier speedController) {
        return run(() -> {
            double speedControl = speedController.getAsDouble() / 2 + 0.8;
            swerveDrive.drive(
                    new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity() * speedControl,
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity() * speedControl),
                    angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                    false, // 🚫 Robot-oriented only (no field-relative)
                    false // Open loop
            );
        });
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
    }
}
