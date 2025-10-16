package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
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
    private static final File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve/neo");
    private static final double MAX_METERS_PER_SECOND = 2;

    private static final double RAMP_RATE = 1.0 / 3.0; //calculate how quickly to ramp to full power (1 / seconds)
    private static final SlewRateLimiter DRIVE_RAMP_Y_LIMITER = new SlewRateLimiter(RAMP_RATE);
    private static final SlewRateLimiter DRIVE_RAMP_X_LIMITER = new SlewRateLimiter(RAMP_RATE);

    private final SwerveDrive swerveDrive;

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;

        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(MAX_METERS_PER_SECOND);
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
     * @return Drive command
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> swerveDrive.drive(
            new Translation2d(
                DRIVE_RAMP_X_LIMITER.calculate(translationX.getAsDouble()) * swerveDrive.getMaximumChassisVelocity(),
                DRIVE_RAMP_Y_LIMITER.calculate(translationY.getAsDouble()) * swerveDrive.getMaximumChassisVelocity()),  
            angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
            false,
            false));
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
    }
}
