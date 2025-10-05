package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class CannonSubsystem extends SubsystemBase {
    private static final double DURATION = 0.25; // seconds

    private final Solenoid solenoid = new Solenoid(
        RobotContainer.PNEUMATIC_CONTROLLER_ID, 
        PneumaticsModuleType.REVPH,
         RobotContainer.CANNON_PNEUMATIC_CHANNEL_ID);

    public void fire() {
        solenoid.set(true);
    }

    public void stopFiring() {
        solenoid.set(false);
    }

    public Command fireCommand(Command revolverNextSlot) {
        return new SequentialCommandGroup(
            new InstantCommand(this::fire, this),
            new WaitCommand(DURATION),
            new InstantCommand(this::stopFiring, this),
            revolverNextSlot
        );
    }
}
