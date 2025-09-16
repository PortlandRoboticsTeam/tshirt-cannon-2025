package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SolinoidSubsystem extends SubsystemBase{
    private boolean saftey = true;
    private final Solenoid solenoid;
    
    public SolinoidSubsystem(int modualID, int channel){
        solenoid = new Solenoid(modualID ,PneumaticsModuleType.REVPH, channel);
    }

    public Solenoid getSolenoid() {
        return solenoid;
    }

    public boolean getSafety() {
        return saftey;
    }

    public void setSaftey(boolean saftey) {
        this.saftey = saftey;
    }
    public void setSolinoidState(boolean state){
        solenoid.set(state);
    }

    public void toggleSaftey() {
        saftey = !saftey;
    } 
}
