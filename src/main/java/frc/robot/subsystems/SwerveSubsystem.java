package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.BooleanSupplier;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.SwerveDrive;
import swervelib.imu.SwerveIMU;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase{
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve/neo");
    SwerveDrive swerveDrive;
    Limelight limelight = new Limelight();
    double speedControl = 1;
    public SwerveSubsystem(){
        try {
          SwerveDriveTelemetry.verbosity = Constants.telemetryVerbosity;
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.maximumSpeed);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        setupPathPlanner();
    }
    
     /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX,DoubleSupplier speedController)
  {
    return run(() -> {
      // Make the robot move
      speedControl = speedController.getAsDouble()*2+.6;
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity()*getSpeedControl(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()*getSpeedControl()),
        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
        true,
        false);
    });
  }

  private double getSpeedControl() {
      return speedControl;
    }

  // this overload was added by michael diorio
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX, BooleanSupplier isFieldOriented,DoubleSupplier speedController)
  {
    return run(() -> {
      speedControl = speedController.getAsDouble()/2+.8;
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity()*getSpeedControl(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()*getSpeedControl()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        isFieldOriented.getAsBoolean(),
                        false);
    });
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public Command getResetGyro() {
    return new InstantCommand(()->swerveDrive.zeroGyro(),this);
  }

  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
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
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,// #TODO What the refrigerator is going on?!?!
                  swerveDrive.kinematics.toSwerveModuleStates(new ChassisSpeeds(speedsRobotRelative.vxMetersPerSecond, speedsRobotRelative.vyMetersPerSecond, -speedsRobotRelative.omegaRadiansPerSecond)),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(1.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(1.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

   /**
   * get the pose 2D object
   *
   * @return
   */
   public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

   /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

   /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

   /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose)
  {
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
  public SwerveDrive getDriveTrain(){
    return swerveDrive;
  }
  public Command vishionDriveR(){
    return run(() -> {
      // Make the robot move
      //SmartDashboard.putNumber("visOut", limelight.getx());
      swerveDrive.drive(new Translation2d(limelight.getx()-RobotContainer.getController().getRawAxis(1),
                                          limelight.gety()-RobotContainer.getController().getRawAxis(0)),
                                          MathUtil.applyDeadband(RobotContainer.getController().getRawAxis(2), Constants.DEADBAND),//limelight.getAngle(),
                        false,
                        false);
    });
  }
  public Command vishionDriveL(){
    return run(() -> {
      // Make the robot move
      //SmartDashboard.putNumber("visOut", limelight.getx());
      swerveDrive.drive(new Translation2d(-limelight.getyL()-RobotContainer.getController().getRawAxis(1),
      limelight.getxL()-RobotContainer.getController().getRawAxis(0)
      ),MathUtil.applyDeadband(RobotContainer.getController().getRawAxis(2), Constants.DEADBAND),
                                          
                        false,
                        false);
    });

  }
  public Command vishionDrivePickup(){
    return run(() -> {
      // Make the robot move
      //SmartDashboard.putNumber("visOut", limelight.getx());
      swerveDrive.drive(new Translation2d(-limelight.getPickupx()-RobotContainer.getController().getRawAxis(1),
                                          limelight.getPickupy()-RobotContainer.getController().getRawAxis(0)),
                                          MathUtil.applyDeadband(RobotContainer.getController().getRawAxis(2), Constants.DEADBAND),//limelight.getAngle(),
                        false,
                        false);
    });
  }
  public SwerveIMU getGyro() {
    return swerveDrive.getGyro();
  }
}