package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DoubleSolinoidSubsystem extends SubsystemBase{
    private boolean saftey = true;
    private DoubleSolenoid solenoid;
    
    public DoubleSolinoidSubsystem(int modualID, int channel){
        solenoid = new DoubleSolenoid(modualID, PneumaticsModuleType.REVPH,channel,15);
    }

    public DoubleSolenoid getSolenoid() {
        return solenoid;
    }

    public void setSolenoid(DoubleSolenoid solenoid) {
        this.solenoid = solenoid;
    }
    
    public boolean getSafety() {
        return saftey;
    }

    public void setSaftey(boolean saftey) {
        this.saftey = saftey;
    }
    public void setSolinoidState(Value value){
        solenoid.set(value);
    }

    public void toggleSaftey() {
        saftey = !saftey;
    } 
}
