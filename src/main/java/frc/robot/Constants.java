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
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    
  }

  public static final double DEADBAND = 0.1;
  public static final double maximumSpeed = Units.feetToMeters(4);// TODO max speed
  public static final TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.LOW;
public static final int servoID = 0;
    
    public static class GrabberConstants {
      public static final int coralMotorID = 18;
      public static final int algaeMotorID = 19;
      public static final double coralSpeed = 0.2;
      public static final double algaeSpeed = 0.35;
      public static final MotorType algaeMotorType = MotorType.Talon;
      public static final MotorType coralMotorType = MotorType.Talon;
      public static final double sencerdelay = 0.2;
    }
    public static class ArmConstants {
      public static final double manualControlJoystickDeaband = 0.1;
      public static final boolean useBounds = false;
  
      public static final double shoulderOffset = 27-60, shoulderMin =  0, shoulderMax = 184;
      public static final int    shoulder1ID    = 13, shoulder2ID = 14;
      public static final int shoulderEncoderID  = 0;
      public static final MotorType shoulderType = MotorType.Talon;
      public static final EncoderType shoulderEncoderType = EncoderType.DutyCycle;
  
  
      public static final double wristOffset = 0, wristMin =  0, wristMax = 0;
      public static final int wristID  = 16;
      public static final int wristEncoderID  = 17;
      public static final MotorType wristType = MotorType.Talon;
      public static final EncoderType wristEncoderType = EncoderType.CANCoder;
  
  
  
      public static final double telescopeOffset = 0, telescopeMin =  0, telescopeMax = 744;
      public static final int telescopeID  = 15;
      public static final int telescopeEncoderID  = 19;
      public static final int greenThreshold = 30000;
      public static final MotorType telescopeType = MotorType.Talon;
      public static final EncoderType telescopeEncoderType = EncoderType.CANCoder;
      public static final double telescopeCalibrationSpeed = -.2;
  
      public static final ArmPosition positionCommandCompletionTolerance = new ArmPosition(5, .2, 1, "tolerance (not a position)");
  
      public static final ArmPosition[] positions = {
      
        new ArmPosition(50, -357,0, "Rest: 0"), 
        new ArmPosition(60, -330,0, "Rest: 1"),   
    };    
  }
}
