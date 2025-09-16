package frc.robot.commands; //Defines the package for this command class

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Joint;

public class JointToPosition extends Command {
  private final Joint joint;

  public JointToPosition(Joint pJoint) {
    joint = pJoint; 
    addRequirements(pJoint);
  }

  @Override
  public void execute() {
    if (joint.isPIDEnabled())
      if (joint.getEncoder().isConnected()) {
        double output = joint.getController().calculate(joint.getAngleDegrees(), joint.getSetpoint());
        joint.setSpeed(output);
      } else {
        joint.stop();
      }

  }

  @Override
  public void end(boolean interrupted) {
    joint.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}