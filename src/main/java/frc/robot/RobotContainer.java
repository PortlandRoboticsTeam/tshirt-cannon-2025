package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.*;

/**
 * Configures button mappings and subsystems for the robots.
 */
public class RobotContainer {
  private static final double DEADBAND = 0.1;

  // Can and Pneumatic IDs
  public static final int SHOULDER_MOTOR_ID = 12;
  public static final int SHOULDER_ENCODER_ID = 15;
  public static final int ELBOW_MOTOR_ID = 11;
  public static final int ELBOW_ENCODER_ID = 18;
  public static final int REVOLVER_MOTOR_ID = 13;
  public static final int REVOLVER_ENCODER_ID = 17;
  
  public static final int PNEUMATIC_CONTROLLER_ID = 16;
  public static final int CANNON_PNEUMATIC_CHANNEL_ID = 6;
  public static final int HORN_PNEUMATIC_CHANNEL_ID = 7;

  // Subsystems
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final RevolverSubsystem revolver = new RevolverSubsystem();
  private final HornSubsystem horn = new HornSubsystem();
  private final CannonSubsystem tCannon = new CannonSubsystem();

  private final CommandPS4Controller controller = new CommandPS4Controller(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivebase.setDefaultCommand(drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(controller.getLeftY(), DEADBAND),
        () -> -MathUtil.applyDeadband(controller.getLeftX(), DEADBAND),
        () -> -MathUtil.applyDeadband(controller.getRightX(), DEADBAND)));

    // Both the cannon and horn are only activated when the safety (L1) is held
    controller.R1().and(controller.L1()).onTrue(horn.activateHorn());
    controller.R1().and(controller.L1()).onFalse(horn.stopHorn());

    //after firing, the cannon will automatically revolve to the next slot
    controller.R2().and(controller.L1()).onTrue(tCannon.fireCommand(revolver.nextSlot()));

    // Reload the revolver to the next slot when the cross button is pressed
    controller.cross().onTrue(revolver.nextSlot());
    controller.circle().onTrue(revolver.stop());

    // D-Pad left/right to fully extend or retract the arm, stop movement by pressing up/down briefly
    controller.povLeft().onTrue(arm.retract());
    controller.povRight().onTrue(arm.extend());

    // holding up/down on the D-Pad will extend/retract the arm, releasing will stop
    controller.povUp().onTrue(arm.extend());
    controller.povUp().onFalse(arm.stop());
    controller.povDown().onTrue(arm.retract());
    controller.povDown().onFalse(arm.stop());
  }
}
