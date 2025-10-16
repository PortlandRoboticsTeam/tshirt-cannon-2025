package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class HornSubsystem extends SubsystemBase {
    private final Solenoid solenoid = new Solenoid(
        RobotContainer.PNEUMATIC_CONTROLLER_ID, 
        PneumaticsModuleType.REVPH, 
        RobotContainer.HORN_PNEUMATIC_CHANNEL_ID);

    public Command activateHorn() {
        return runOnce(() -> solenoid.set(true));
    }

    public Command stopHorn() {
        return runOnce(() -> solenoid.set(false));
    }
    
}
