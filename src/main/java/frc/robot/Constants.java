// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ArmPosition;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.subsystems.Motor.MotorType;
import frc.robot.subsystems.Encoder.EncoderType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double DEADBAND = 0.1;
  public static final double maximumSpeed = Units.feetToMeters(4);// TODO max speed
  public static final TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.LOW;
  public static final int servoID = 0;

  public static class ArmConstants {
    public static final double manualControlJoystickDeaband = 0.1;
    public static final boolean useBounds = false;

    public static final double shoulderOffset = 27 - 60, shoulderMin = 0, shoulderMax = 184;
    public static final int shoulder1ID = 13, shoulder2ID = 14;
    public static final int shoulderEncoderID = 0;
    public static final MotorType shoulderType = MotorType.Talon;
    public static final EncoderType shoulderEncoderType = EncoderType.DutyCycle;

    public static final ArmPosition[] positions = {
        new ArmPosition(50, -357, 0, "Rest: 0"),
        new ArmPosition(60, -330, 0, "Rest: 1"),
    };
  }
}
