package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

/**
 * A slew rate limiter that limits acceleration/deceleration rates separately.
 */
public class AsymmetricSlewRateLimiter {
  private final double accelRate;
  private final double decelRate;

  private double lastValue = 0.0;
  private double lastTime = Timer.getFPGATimestamp();

  public AsymmetricSlewRateLimiter(double accelRate, double decelRate) {
    if (accelRate < 0 || decelRate < 0) {
      throw new IllegalArgumentException("Rates must be non-negative");
    }
    this.accelRate = accelRate;
    this.decelRate = decelRate;
  }

  public double calculate(double input) {
    double now = Timer.getFPGATimestamp();
    double deltaTime = Math.max(1e-6, now - lastTime);
    lastTime = now;
  
    double cur = lastValue;
  
    // Magnitudes and signs
    double currentMagnitude = Math.abs(cur);
    double targetMagnitude = Math.abs(input);
    double currentSign = (cur == 0.0) ? 0.0 : Math.signum(cur);
    double targetSign = (input == 0.0) ? 0.0 : Math.signum(input);
  
    // If sign differs (and both non-zero), that is a direction reversal -> treat as decel (braking).
    final boolean signReversal = (cur != 0.0 && input != 0.0 && currentSign != targetSign);
  
    double rate;
    if (signReversal) {
      rate = decelRate;
    } else {
      rate = (targetMagnitude > currentMagnitude) ? accelRate : decelRate;
    }
  
    double maxDelta = rate * deltaTime;
  
    // Move the magnitude toward the target magnitude (limited by maxDelta)
    double magnitudeDelta = targetMagnitude - currentMagnitude;
    double magnitudeChange = Math.copySign(Math.min(Math.abs(magnitudeDelta), maxDelta), magnitudeDelta);
    double newMagnitue = currentMagnitude + magnitudeChange;

    double newSign;
    if (newMagnitue == 0.0                                      //magnitude reached zero
        || (currentMagnitude == 0.0 && targetMagnitude > 0.0)   //accelerating from zero
        || targetMagnitude > currentMagnitude) {                //increasing magnitude
      newSign = targetSign;
    } else {
      newSign = currentSign;                                    //decelerating
    }
  
    lastValue = Math.copySign(newMagnitue, newSign);
    return lastValue;
  }
  
}
