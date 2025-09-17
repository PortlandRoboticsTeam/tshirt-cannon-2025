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
import frc.robot.subsystems.Encoder.EncoderType;
import frc.robot.subsystems.motors.Motor.MotorType;

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
  public HornSubsystem horn = new HornSubsystem(16, 7, 15);
  public CannonSubsystem tCannon = new CannonSubsystem(16, 6);

  private InstantCommand[] goToPositionCommand = new InstantCommand[2];

  private final CommandPS4Controller driverXbox = new CommandPS4Controller(0);

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
   * Binds the controls on the xbox controller to commands on the robot.
   */
  private void configureBindings() {
    driverXbox.button(5).onTrue(zeroGyro);

    // Both the cannon and horn are only activated when the safety (L1) is held
    driverXbox.R1().and(driverXbox.L1()).whileTrue(horn.generateHoldCommand());
    driverXbox.R2().and(driverXbox.L1()).onTrue(tCannon.generateFireCommand());

    driverXbox.cross().onTrue(new InstantCommand((() -> revolver.nextSlot())));

    driverXbox.povDown().whileTrue(new RunCommand(() -> manualArmControl(true), shoulder, elbow));
    driverXbox.povUp().whileTrue(new RunCommand(() -> manualArmControl(false), shoulder, elbow));
    
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
