// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.*;

/**
 * Configures button mappings and subsystems for the robots.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  public Command zeroGyro = drivebase.getResetGyro();

  // private final JointSubsystem elbow = new JointSubsystem(11, 6, false);
  // private final JointSubsystem shoulder = new JointSubsystem(12, 15, true);
  private final RevolverSubsystem revolver = new RevolverSubsystem(13, 17);
  public HornSubsystem horn = new HornSubsystem(16, 7, 15);
  public CannonSubsystem tCannon = new CannonSubsystem(16, 6);

  private final CommandPS4Controller controller = new CommandPS4Controller(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // configureArmSystems();
    configureBindings();

    Command driveCommand = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(controller.getLeftY(), Constants.DEADBAND),
        () -> -MathUtil.applyDeadband(controller.getLeftX(), Constants.DEADBAND),
        () -> -MathUtil.applyDeadband(controller.getRightX(), Constants.DEADBAND),
        () -> 0);
    drivebase.setDefaultCommand(driveCommand);
  }

  /**
   * Binds the controls on the xbox controller to commands on the robot.
   */
  private void configureBindings() {
    controller.button(5).onTrue(zeroGyro);

    // Both the cannon and horn are only activated when the safety (L1) is held
    controller.R1().and(controller.L1()).whileTrue(horn.generateHoldCommand());
    controller.R2().and(controller.L1()).onTrue(tCannon.generateFireCommand());

    // Reload the revolver to the next slot when the cross button is pressed
    // driverXbox.cross().onTrue(new InstantCommand((() -> revolver.nextSlot())));
    controller.cross().onTrue(new InstantCommand(() -> {
      revolver.nextSlot();
    }));
    controller.circle().onTrue(new InstantCommand(() -> {
      revolver.stop();
    }));

    // // allow the arm to move up and down with the D-pad
    // driverXbox.povDown().whileTrue(new RunCommand(() -> manualArmControl(true),
    // shoulder, elbow));
    // driverXbox.povUp().whileTrue(new RunCommand(() -> manualArmControl(false),
    // shoulder, elbow));

    // // preset positions for the arm in resting or firing modes
    // driverXbox.povLeft().onTrue(
    // new InstantCommand(() -> {
    // shoulder.setSetpoint(ArmConstants.positions[0].getShoulderPos() / 360.0);
    // elbow.setSetpoint(ArmConstants.positions[0].getElbowPos() / 360.0);
    // }, shoulder, elbow));

    // driverXbox.povRight().onTrue(
    // new InstantCommand(() -> {
    // shoulder.setSetpoint(ArmConstants.positions[1].getShoulderPos() / 360.0);
    // elbow.setSetpoint(ArmConstants.positions[1].getElbowPos() / 360.0);
    // }, shoulder, elbow));
  }

  // private void configureArmSystems() {
  // // Apply bounds
  // shoulder.applyBounds(ArmConstants.shoulderMin / 360.0,
  // ArmConstants.shoulderMax / 360.0);
  // elbow.applyBounds(ArmConstants.elbowMin / 360.0, ArmConstants.elbowMax /
  // 360.0);

  // // Set encoder offsets (degrees -> rotations)
  // shoulder.getEncoder().setOffset(ArmConstants.shoulderOffset / 360.0);
  // elbow.getEncoder().setOffset(ArmConstants.elbowOffset / 360.0);

  // // Commented out for safety: don't move the joints automatically at startup
  // // shoulder.setSetpoint(shoulder.getAngleDegrees() / 360.0);
  // // elbow.setSetpoint(elbow.getAngleDegrees() / 360.0);
  // }

  // public void manualArmControl(boolean reversed) {
  // double delta = (reversed ? -1 : 1) * 2.0 / 360.0; // ~2 degrees per press
  // shoulder.setSetpoint(shoulder.getSetpoint() + delta);
  // elbow.setSetpoint(elbow.getSetpoint() + delta);
  // }

}
