package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HornSubsystem extends SubsystemBase {
    private final Solenoid solenoid;

    public HornSubsystem(int moduleID, int forwardChannel) {
        solenoid = new Solenoid(moduleID, PneumaticsModuleType.REVPH, forwardChannel);
    }

    public void activateHorn() {
        solenoid.set(true);
    }

    public void stopHorn() {
        solenoid.set(false);
    }
    
}
