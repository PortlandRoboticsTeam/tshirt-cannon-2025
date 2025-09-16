package frc.robot.commands; //Defines the package for this command class

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.Joint;
import frc.robot.subsystems.Telescope;

public class ArmPositionCommandGenerator{
  //Subsystem refrences for the arm
  private Joint shoulder1;
  private Joint shoulder2;
  private Telescope telescope;
  private Joint wrist;

  //stores allowable position tolerences 
  private final ArmPosition tol = ArmConstants.positionCommandCompletionTolerance;

  //Constructor: injects the specific joints to be controlled
  public ArmPositionCommandGenerator(Joint s1, Joint s2, Telescope t, Joint w){
    shoulder1=s1; 
    shoulder2=s2; 
    telescope=t; 
    wrist=w;
  }

  //Builds and returns a Command that moves the arm components to its target Armposition
  public Command compileCommand(ArmPosition pos){
    //Command to move bothe shoulder joints (runs in parallel via alongwith)
    Command shoulderCommand =  shoulder1.getGoToCommand(pos.getShoulderPos (), tol.getShoulderPos ())
                    .alongWith(shoulder2.getGoToCommand(pos.getShoulderPos (), tol.getShoulderPos ()));
    //Command to move the wrist
    Command wristCommand =     wrist    .getGoToCommand(pos.getWristPos    (), tol.getWristPos    ());
    //Command to move the telescope
    Command telescopeCommand = telescope.getGoToCommand(pos.getTelescopePos(), tol.getTelescopePos());


    //run the wrist command first, then sequence shoulder/ telescope commands
    // in an order depending on if the new shoulder position is higher or lower
    return wristCommand.andThen(
      pos.getShoulderPos() > shoulder1.getAngleDegrees() ?
        shoulderCommand.andThen(telescopeCommand) :
        telescopeCommand.andThen(shoulderCommand)
    );
  }
}
