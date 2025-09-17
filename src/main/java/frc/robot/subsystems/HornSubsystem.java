package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HornSubsystem extends SubsystemBase {
    private final DoubleSolenoid solenoid;

    public HornSubsystem(int moduleID, int forwardChannel, int reverseChannel) {
        solenoid = new DoubleSolenoid(moduleID, PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);
    }

    public void activateHorn() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void stopHorn() {
        solenoid.set(DoubleSolenoid.Value.kOff);
    }

    /** 
     * Returns a command that keeps the horn on while the trigger is active.
     * The scheduler automatically ends the command when the trigger is released.
     */
    public Command generateHoldCommand() {
        return new RunCommand(this::activateHorn, this)
                .until(() -> false) // will be ended externally by the button trigger
                .andThen(this::stopHorn);
    }
}
