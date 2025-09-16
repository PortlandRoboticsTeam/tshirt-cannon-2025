package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntervalGradient extends SubsystemBase{
  
  private final DoubleSupplier valueSupplier;
  private final double influence;
  private double lastValue;

  /**
   * A system to smoothen <double> input systems
   * @param value a DoubleSupplier used to channel user input
   * @param infl the fraction/second change in output;
   */
  public IntervalGradient(DoubleSupplier value, double infl){
    valueSupplier = value;
    influence = infl*.05;
  }

  public double getValue(){ return lastValue; }
  public DoubleSupplier getValueSupplier(){ return ()->getValue(); }

  public void periodic(){
    lastValue = (valueSupplier.getAsDouble()-lastValue)*influence + lastValue;
  }
  
}
