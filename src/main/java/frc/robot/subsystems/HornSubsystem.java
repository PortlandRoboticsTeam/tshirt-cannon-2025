package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class HornSubsystem extends SubsystemBase {
    private final Solenoid solenoid = new Solenoid(
        RobotContainer.HORN_MOTOR_ID, PneumaticsModuleType.REVPH, RobotContainer.HORN_PNEUMATIC_CHANNEL_ID);

    public Command activateHorn() {
        return new InstantCommand(() -> solenoid.set(true));
    }

    public Command stopHorn() {
        return new InstantCommand(() -> solenoid.set(false));
    }
    
}
