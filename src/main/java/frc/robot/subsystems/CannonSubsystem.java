package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CannonSubsystem extends SubsystemBase {
    private final Solenoid solenoid;
    private static final double DURATION = 1; // seconds

    public CannonSubsystem(int moduleID, int channel) {
        solenoid = new Solenoid(moduleID, PneumaticsModuleType.REVPH, channel);
    }

    public void fire() {
        solenoid.set(true);
    }

    public void stopFiring() {
        solenoid.set(false);
    }

    public Command generateFireCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(this::fire, this),
            new WaitCommand(DURATION),
            new InstantCommand(this::stopFiring, this)
        );
    }
}
