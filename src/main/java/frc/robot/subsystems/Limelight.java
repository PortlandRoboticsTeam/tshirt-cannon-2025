package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
  NetworkTableInstance tables = NetworkTableInstance.getDefault();
  NetworkTable right = tables.getTable("limelight");
  NetworkTable left = tables.getTable("limelight-a");
  boolean useRight = false;

  public void setlimeLight(boolean useRight){
    this.useRight = useRight;
  }
  public double getx() {
    double p = .11;
    left.getEntry("pipeline").setNumber(0);
    return -left.getEntry("tx").getDouble(0)*p;
  }

  public double gety() {
    double p = .11;
    left.getEntry("pipeline").setNumber(0);
    return left.getEntry("ty").getDouble(0)*p;
  }

  @Override
  public void periodic() {
      super.periodic();
  }
public double getAngle() {
  double p = .18 ;
  return (useRight?right:left).getEntry("ty").getDouble(0)*-p;
}
public double getxL() {
  double p = .11;
  return -right.getEntry("tx").getDouble(0)*p;
}
public double getyL() {
  double p = .11;
  return right.getEntry("ty").getDouble(0)*p;
}
public double getPickupx(){
  double p = .12;
    left.getEntry("pipeline").setNumber(1);
    return left.getEntry("tx").getDouble(0)*p;
}
public double getPickupy(){
  double p = .12;
    left.getEntry("pipeline").setNumber(1);
    // SmartDashboard.putNumber("pipline", left.getEntry("pipline"));
    return left.getEntry("ty").getDouble(0)*p;
}
}
