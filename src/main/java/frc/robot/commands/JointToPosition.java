package frc.robot.commands; //Defines the package for this command class

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Encoder;
import frc.robot.subsystems.Joint;

public class JointToPosition extends Command{
    //refrences to the joint & encoder subsystem
    Joint m_joint;
    Encoder encoder;


    //Constructor that accepts the join to be controlled
    public JointToPosition(Joint joint) {
    m_joint = joint; //get the specific joint 
    encoder = joint.getEncoder(); //get the encoder from the specified joint

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_joint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_joint.isPIDEnabled())
      if (encoder.isConnected()) {
        double output = m_joint.getController().calculate(m_joint.getAngleDegrees(), m_joint.getSetpoint());
        m_joint.setSpeed(output);
      }else{
        m_joint.stop();
      }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_joint.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}