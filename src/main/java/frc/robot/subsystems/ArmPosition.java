package frc.robot.subsystems;

public class ArmPosition {

  private final double shoulderPos;
  private final double elbowPos;
  private final String name;

  public ArmPosition(double sPos, double tPos, String nme) {
    shoulderPos = sPos;
    elbowPos = tPos;
    name = nme;
  }

  public double getShoulderPos() {
    return shoulderPos;
  }

  public double getElbowPos() {
    return elbowPos;
  }

  public String getName() {
    return name;
  }
}