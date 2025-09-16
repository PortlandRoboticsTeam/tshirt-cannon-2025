package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SolinoidSubsystem;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class FireCannon extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SolinoidSubsystem tshirtCannon;
    private Timer fireTimer;
    private Solenoid solenoid;

    public FireCannon(SolinoidSubsystem tshirtCannon){
        this.tshirtCannon = tshirtCannon;
        this.solenoid = tshirtCannon.getSolenoid();
        addRequirements(tshirtCannon);
        fireTimer = new Timer();
    }
    

    @Override
    public void initialize(){
        fireTimer.start();
        System.out.println("fireing " + !tshirtCannon.getSafety());
        if (!tshirtCannon.getSafety()) {
            tshirtCannon.setSolinoidState(true);
        }
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        if(fireTimer.get()>=0.25){
            return true;
        }
        else return false;
    }

    @Override
    public void end(boolean interrupted){
        solenoid.set(false);
        fireTimer.stop();
        fireTimer.reset();
    }
}