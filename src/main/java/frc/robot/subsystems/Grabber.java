package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class Grabber extends SubsystemBase {
    DigitalInput sensor = new DigitalInput(4);
    private Motor coralMotor = new Motor(GrabberConstants.coralMotorID,GrabberConstants.coralMotorType);
    boolean grabbing = false;
    State state = State.stop;
    Timer t = new Timer();
    double grabTime =0;
    public enum State{
        forward,
        reverse,
        stop
    }


    public Grabber(){

    }

    public void grab(){
        grabbing = true;
        state = State.forward;
    }
    public void grabReverse(){
        grabbing = true;
        state = State.reverse;
    }

    public void release(){
        grabbing = false;
        state =State.forward;
    }
    public void stop(){
        state = State.stop;
    }
    public Motor getCoralMotor(){
        return coralMotor;
    }
    // public Motor getAlgaeMotor(){
    //     return algaeMotor;
    // }
    public void periodic(){
        switch (state) {
            case forward:
                coralMotor.set(GrabberConstants.coralSpeed);
                break;
            case reverse:
                coralMotor.set(-GrabberConstants.coralSpeed);
                break;
            case stop:
                coralMotor.set(0);
                break;
        
            default:
                break;
        }
        if (grabbing&&!sensor.get()) {
           grabTime = Timer.getMatchTime()-.25;

        // if (grabbing&&!sensor.get()) {
        //     grabTime = Timer.getMatchTime();
            
        }
        if (grabTime<Timer.getMatchTime()&&Timer.getMatchTime()>grabTime-GrabberConstants.sencerdelay) {
            grabbing = !grabbing;
            state = State.stop;
            grabTime = 100000;
        }
        if (!grabbing&&sensor.get()) {
            grabTime = Timer.getMatchTime()-.25;
            //state = State.stop;
        }
        
        
        SmartDashboard.putBoolean("coralSensor", sensor.get());
        SmartDashboard.putBoolean("grabbing", grabbing);
    }
}
