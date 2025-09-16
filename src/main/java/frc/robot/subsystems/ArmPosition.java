package frc.robot.subsystems;

public class ArmPosition {

  private double shoulderPos;
  private double telescopePos;
  private double wristPos;
  private String name;

  public ArmPosition (double sPos, double tPos, double wPos, String _name){
    shoulderPos = sPos;
    telescopePos = tPos;
    wristPos = wPos;
    name = _name;
  }

  public double getShoulderPos (){ return shoulderPos;  }
  public double getTelescopePos(){ return telescopePos; }
  public double getWristPos    (){ return wristPos;     }

  public String getName() { return name; }
}