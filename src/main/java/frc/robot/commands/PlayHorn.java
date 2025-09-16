package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DoubleSolinoidSubsystem;
import frc.robot.subsystems.SolinoidSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PlayHorn extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DoubleSolinoidSubsystem horn;
    private Timer fireTimer;
    private DoubleSolenoid solenoid;

    public PlayHorn(DoubleSolinoidSubsystem horn){
        this.horn = horn;
        this.solenoid = horn.getSolenoid();
        addRequirements(horn);
        fireTimer = new Timer();
    }
    

    @Override
    public void initialize(){
        fireTimer.start();
        System.out.println("fireing " + !horn.getSafety());
        if (!horn.getSafety()) {
            horn.setSolinoidState(Value.kForward);
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
        solenoid.set(Value.kOff);
        fireTimer.stop();
        fireTimer.reset();
    }
}