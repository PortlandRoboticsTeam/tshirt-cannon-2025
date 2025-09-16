package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.imu.SwerveIMU;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private double speedControl = 1.0;

    public SwerveSubsystem() {
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve/neo");
            SwerveDriveTelemetry.verbosity = Constants.telemetryVerbosity;
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.maximumSpeed);
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
            speedControl = speedController.getAsDouble() / 2 + 0.8;
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

    /** Reset gyro heading to zero. */
    public Command getResetGyro() {
        return new InstantCommand(() -> swerveDrive.zeroGyro(), this);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveIMU getGyro() {
        return swerveDrive.getGyro();
    }

    public SwerveDrive getDriveTrain() {
        return swerveDrive;
    }
}
