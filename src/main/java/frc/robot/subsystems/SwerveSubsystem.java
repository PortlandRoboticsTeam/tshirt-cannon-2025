package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase {
  private static final File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  private static final double MAX_METERS_PER_SECOND = 4;

  // limit rotation to 50% max motor speed
  private static final double MAX_ROTATION_PERCENT = .75;

  // How quickly to ramp to full power (1 / seconds)
  private static final double DRIVE_RAMP_ACCELERATE_RATE = 1.0 / 2.0;   // 2 seconds to full
  private static final double DRIVE_RAMP_DECELERATE_RATE = 1.0 / 0.5;   // .5 second to zero

  private final AsymmetricSlewRateLimiter DRIVE_RAMP_X_LIMITER
    = new AsymmetricSlewRateLimiter(DRIVE_RAMP_ACCELERATE_RATE, DRIVE_RAMP_DECELERATE_RATE);
  private final AsymmetricSlewRateLimiter DRIVE_RAMP_Y_LIMITER
    = new AsymmetricSlewRateLimiter(DRIVE_RAMP_ACCELERATE_RATE, DRIVE_RAMP_DECELERATE_RATE);
      
  // ramp up to full speed in 1 second, decelerate in 1/4 second
  private final AsymmetricSlewRateLimiter ROTATION_RAMP_LIMITER
    = new AsymmetricSlewRateLimiter(1.0, 1.0 / 0.25); 

  private final SwerveDrive swerveDrive;
  private boolean fieldRelativeDrive;
  private boolean sportsMode;

  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(MAX_METERS_PER_SECOND);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier driveGradientX, DoubleSupplier driveGradientY, DoubleSupplier angularRotationX) {
    return run(() -> {
      swerveDrive.drive(
        new Translation2d(
              DRIVE_RAMP_X_LIMITER.calculate(driveGradientX.getAsDouble())
                 * swerveDrive.getMaximumChassisVelocity() * (sportsMode ? 1.0 : 0.5),
              DRIVE_RAMP_Y_LIMITER.calculate(driveGradientY.getAsDouble())
                 * swerveDrive.getMaximumChassisVelocity() * (sportsMode ? 1.0 : 0.5)),
        ROTATION_RAMP_LIMITER.calculate(angularRotationX.getAsDouble())
           * swerveDrive.getMaximumChassisAngularVelocity() * MAX_ROTATION_PERCENT,
        fieldRelativeDrive,
        false);
    });
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public Command getResetGyro() {
    return runOnce(() -> swerveDrive.zeroGyro());
  }

  public Command setFieldRelativeDrive(boolean rel) {
    return runOnce(() -> this.fieldRelativeDrive = rel);
  }

  public Command setSportsMode(boolean mode) {
    return runOnce(() -> this.sportsMode = mode);
  }

  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
          // Rotation PID constants
          ),
          config,
          () -> {
            // flip the path being followed to the red side of the field.
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * get the pose 2D object
   *
   * @return
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose
   * estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to
   * resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }

  public void updateOdometry() {
    swerveDrive.updateOdometry();
  }

  public SwerveDrive getDriveTrain() {
    return swerveDrive;
  }

}