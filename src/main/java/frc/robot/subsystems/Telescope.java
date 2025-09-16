package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Encoder.EncoderType;
import frc.robot.subsystems.Motor.MotorType;

public class Telescope extends Joint{
  // Constructor to initialize the Joint subsystem
  ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private boolean isCalibrating = false;
  Servo s = new Servo(Constants.servoID);
    boolean locked = false;
  
  
    public Telescope(int jointNum, int motorID, int encoderID, boolean inverted, int defaultSetpoint, double kP, double kI, double kD,MotorType type, EncoderType eType) {
      super(jointNum,motorID,encoderID,inverted,defaultSetpoint,kP,kI,kD,type,eType);
      getPID().disableContinuousInput();
      // calibrateWithColors();
    }
  
    public void calibrateWithColors() {
      disablePID();
      isCalibrating=true;
      setSpeed(ArmConstants.telescopeCalibrationSpeed);
    }
  
    // Method to get the angle in degrees based on encoder position
    @Override
    public double getAngleDegrees() {
        return getEncoder().getValue();
    }
  
    @Override
    public void periodic(){
      super.periodic();
      if(isCalibrating) setSpeed(ArmConstants.telescopeCalibrationSpeed);
      if(colorSensor.getGreen()<ArmConstants.greenThreshold && isCalibrating){
        getEncoder().setOffsetTo(.08);
        setSetpoint(0);
        stop();
        enablePID();
        isCalibrating = false;
        //low pos should 24.5in from the crossbeam on the mounting bracket to the end
      }
      setservo();
      SmartDashboard.putNumber("servo", s.get());
      SmartDashboard.putBoolean("iscalibrating", isCalibrating);
      SmartDashboard.putBoolean("is pid enabled", isPIDEnabled());
      SmartDashboard.putNumber("getangledegrees", getEncoder().getValue());
      SmartDashboard.putNumber("Color Green Value", colorSensor.getGreen());
      SmartDashboard.putNumber("Color Red Value", colorSensor.getRed());
      SmartDashboard.putNumber("Color Blue Value", colorSensor.getBlue());
    }
  
  public boolean isCalibrating() {
    return isCalibrating;
  }
  public void setservo(){
    s.set(locked?.25:0);
}
  public void lock(){
    locked = true;
  }
  public void unlock(){
    locked = false;
  }
  @Override
  public void initialize(){
    unlock();
  }
}