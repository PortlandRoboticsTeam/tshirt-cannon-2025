// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.FireCannon;
import frc.robot.commands.PlayHorn;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Encoder.EncoderType;
import frc.robot.subsystems.Motor.MotorType;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  public Command zeroGyro = drivebase.getResetGyro();

  private final JointSubsystem elbow = new JointSubsystem(11, 6, false, 0, 0.01, 0.01, 0.00, MotorType.SparkMax,
      EncoderType.CANCoder);
  private final JointSubsystem shoulder = new JointSubsystem(12, 5, true, 0, 0.043, 0.0000, 0.0000, MotorType.SparkMax,
      EncoderType.CANCoder);
  private final RevolverSubsystem revolver = new RevolverSubsystem(13, 1);

  public SolinoidSubsystem tCannon = new SolinoidSubsystem(16, 6);
  public FireCannon fireCannon = new FireCannon(tCannon);

  public Command saftyToggle = new InstantCommand(() -> tCannon.toggleSaftey());
  public Command nextBarrel = new InstantCommand((() -> revolver.nextSlot()));

  public DoubleSolinoidSubsystem horn = new DoubleSolinoidSubsystem(16, 7);
  Command playHorn = new PlayHorn(horn);
  public Command HsaftyToggle = new InstantCommand(() -> horn.toggleSaftey());

  private InstantCommand[] goToPositionCommand = new InstantCommand[2];

  static final CommandPS4Controller driverXbox = new CommandPS4Controller(0);

  public static CommandPS4Controller getController() {
    return driverXbox;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureArmSystems();

    configureBindings();

    Command driveCommand = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(1), Constants.DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(0), Constants.DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(2), Constants.DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(5), .4));

    drivebase.setDefaultCommand(driveCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    driverXbox.cross().onTrue(nextBarrel);
    driverXbox.R2().onTrue(fireCannon);
    driverXbox.L2().onTrue(saftyToggle.alongWith(HsaftyToggle));
    driverXbox.button(5).onTrue(zeroGyro);
    driverXbox.R1().and(driverXbox.L1()).onTrue(playHorn);
    driverXbox.povDown().whileTrue(new InstantCommand(() -> manualArmControl(true)));
    driverXbox.povUp().whileTrue(new InstantCommand(() -> manualArmControl(false)));
    driverXbox.povLeft().onTrue(goToPositionCommand[0]);
  }

  private void configureArmSystems() {
    // Apply bounds
    shoulder.applyBounds(ArmConstants.shoulderMin / 360.0, ArmConstants.shoulderMax / 360.0);
    elbow.applyBounds(ArmConstants.elbowMin / 360.0, ArmConstants.elbowMax / 360.0);

    // Set encoder offsets (degrees -> rotations)
    shoulder.getEncoder().setOffset(ArmConstants.shoulderOffset / 360.0);
    elbow.getEncoder().setOffset(ArmConstants.elbowOffset / 360.0);

    // Commented out for safety: don't move the joints automatically at startup
    // shoulder.setSetpoint(shoulder.getAngleDegrees() / 360.0);
    // elbow.setSetpoint(elbow.getAngleDegrees() / 360.0);

    // Pre-build commands for named positions
    for (int i = 0; i < ArmConstants.positions.length; i++) {
      final int index = i;
      goToPositionCommand[i] = new InstantCommand(() -> {
        shoulder.setSetpoint(ArmConstants.positions[index].getShoulderPos() / 360.0);
        elbow.setSetpoint(ArmConstants.positions[index].getElbowPos() / 360.0);
      });
    }
  }

  public void manualArmControl(boolean reversed) {
    double delta = (reversed ? -1 : 1) * 2.0 / 360.0; // ~2 degrees per press
    shoulder.setSetpoint(shoulder.getSetpoint() + delta); 
    elbow.setSetpoint(elbow.getSetpoint() + delta);
  }

}
