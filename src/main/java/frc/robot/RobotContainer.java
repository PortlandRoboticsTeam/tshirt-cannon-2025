// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FireCannon;
import frc.robot.commands.JointToPosition;
// import frc.robot.commands.JointToPosition;
import frc.robot.commands.PlayHorn;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Encoder.EncoderType;
import frc.robot.subsystems.Motor.MotorType;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  public Command zeroGyro = drivebase.getResetGyro();
  

  //creates the Joints elbow 
  //cut after 0: new int[] {0,45} shoulder: new int[] {0,32}
  //private final Joint wrist = new Joint(0, 10, 7, false, 1,new int[] {0,10}, .0003, .0000, 0.0000);
  private final Joint elbow = new Joint(1, 11, 6, false, 0, .01, .01, 0.00, MotorType.SparkMax, EncoderType.CANCoder);
  private final Joint shoulder = new Joint(2, 12, 5, true, 0, .043, .0000, 0.0000, MotorType.SparkMax, EncoderType.CANCoder);
  
  // Creates the Joints commands
  public JointToPosition shoulderMove = new JointToPosition(shoulder);
  public JointToPosition elbowMove = new JointToPosition(elbow);
  //public JointToPosition wristMove = new JointToPosition(wrist);

  //creates the TCannon subsystems
  public SolinoidSubsystem tCannon= new SolinoidSubsystem(16,6);
  //After 5th variable of revolver: new int[]{0,60,120,180,240,300},
  public final Joint revolver= new Joint(3,13,1,true,0,.02,0.00,0.0001, MotorType.SparkMax, EncoderType.CANCoder);
  //creates TCannon control commands
  public FireCannon fireCannon= new FireCannon(tCannon);
  public JointToPosition revolverControl = new JointToPosition(revolver);
  //creates safty control commands
  public Command saftyOn = new InstantCommand(()->tCannon.setSaftey(true));
  public Command saftyOff = new InstantCommand(()->tCannon.setSaftey(false));
  public Command saftyToggle = new InstantCommand(()->tCannon.toggleSaftey());
  public Command nextBarrel = new InstantCommand((()->revolver.setSetpoint((revolver.getSetpoint()+60)%360)));

  // creates airHorn
   public DoubleSolinoidSubsystem horn = new DoubleSolinoidSubsystem(16,7);
  // creates command for the airHorn
 Command playHorn = new PlayHorn(horn);
   public Command HsaftyToggle = new InstantCommand(()->horn.toggleSaftey());
 


  // joint tuning commands
  Joint tuningJoint = elbow;
  public Command TunerCommand = new InstantCommand(()->tuningJoint.getController().setP(tuningJoint.getController().getP()+.001));
  
    private InstantCommand[] goToPositionCommand = new InstantCommand[2];
  
  
  
    // Replace with CommandPS4Controller or CommandJoystick if needed
    static final CommandPS4Controller driverXbox = new CommandPS4Controller(0);
    public static CommandPS4Controller getController(){
      return driverXbox;
    }
    
  
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
      configureArmSystems();
      // Configure the trigger bindings
      configureBindings();
      Command driveCommand = drivebase.driveCommand(
          // driverLSYGradient.getValueSupplier(),
          // driverLSXGradient.getValueSupplier(),
          // driverRSXGradient.getValueSupplier(),
          () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(1), Constants.DEADBAND),
          () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(0), Constants.DEADBAND),
          () -> - MathUtil.applyDeadband(driverXbox.getRawAxis(2), Constants.DEADBAND),
          () ->  false,//isFieldOriented
          () ->  -MathUtil.applyDeadband(driverXbox.getRawAxis(5),.4)
        );
  
      
      drivebase.setDefaultCommand(driveCommand);
      revolver.setDefaultCommand(revolverControl);
    }
  
    
  
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
     * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings(){
      driverXbox.cross().onTrue(nextBarrel);
      driverXbox.R2().onTrue(fireCannon);
      driverXbox.L2().onTrue(saftyToggle.alongWith(HsaftyToggle));
      driverXbox.button(5).onTrue(zeroGyro);
      driverXbox.R1().and(driverXbox.L1()).onTrue(playHorn);
      driverXbox.povDown().whileTrue(new InstantCommand(()->manualArmControl(true)));
      driverXbox.povUp().whileTrue(new InstantCommand(()->manualArmControl(false)));
      driverXbox.povLeft().onTrue(goToPositionCommand[0]);
    }
  
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
      // An example command will be run in autonomous
      return drivebase.getAutonomousCommand("New Auto");
    }
  
    public void setDriveMode()
    {
      //drivebase.setDefaultCommand();
    }
    private void configureArmSystems(){
        elbow.getPID().enableContinuousInput(0,-360);
        shoulder.setDefaultCommand(new JointToPosition(shoulder));
        elbow.setDefaultCommand(new JointToPosition(elbow));
        if(ArmConstants.useBounds){
          shoulder.applyBounds(ArmConstants.shoulderMin , ArmConstants.shoulderMax );
          // shoulder2.applyBounds(ArmConstants.shoulderMin , ArmConstants.shoulderMax );
          elbow.applyBounds(ArmConstants.telescopeMin, ArmConstants.telescopeMax);
        }
  
        shoulder.getEncoder().setOffset(ArmConstants.shoulderOffset /360);
        
        elbow    .getEncoder().setOffset(ArmConstants.wristOffset    /360);
  
        shoulder.setSetpoint(shoulder.getAngleDegrees());
        // shoulder2.setSetpoint(shoulder2.getAngleDegrees());
        elbow.setSetpoint(elbow.getAngleDegrees());
  
        // defining all the setpoint commands
        for(int i = 0; i<ArmConstants.positions.length; i++){
          ArmPosition thisArmPosition = ArmConstants.positions[i];
          goToPositionCommand[i] = new InstantCommand(()->{
          // wrist    .setSetpoint(thisArmPosition.getWristPos    ());
          shoulder.setSetpoint(thisArmPosition.getShoulderPos ());
          elbow.setSetpoint(thisArmPosition.getTelescopePos());
          SmartDashboard.putString("Arm Setpoint", thisArmPosition.getName());
        });
        NamedCommands.registerCommand(thisArmPosition.getName(), goToPositionCommand[i]);
      }
    }
    public void manualArmControl(boolean reversed){
      if(!reversed){
        shoulder.setSetpoint(shoulder.getSetpoint()+1);
        elbow.setSetpoint(elbow.getSetpoint()+1);
      }
      else{
        shoulder.setSetpoint(shoulder.getSetpoint()-1);
        elbow.setSetpoint(elbow.getSetpoint()-1);
      }

    }

}
