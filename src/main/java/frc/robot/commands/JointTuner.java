package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Joint;

public class JointTuner extends Command{
    Joint joint;
    Direction direction;
    value value;
    static double incrementation = .1;
    public enum Direction{
        inc,
        dec
            }
            public enum value{
                p,
                i,
                d
            }
            public JointTuner(Joint joint,Direction direction,value value){
                this.joint = joint;
                this.direction = direction;
                this.value = value;
            }
            public void initialize(){
                switch (value) {
                    case p:
                        joint.setPIDValue(joint.getPID().getP()+(direction==Direction.dec?incrementation:-incrementation), joint.getPID().getI(), joint.getPID().getD());
            case i:
                joint.setPIDValue(joint.getPID().getP(), joint.getPID().getI(), joint.getPID().getD());
            case d:
                joint.setPIDValue(joint.getPID().getP(), joint.getPID().getI(), joint.getPID().getD());
        }
    }
}
