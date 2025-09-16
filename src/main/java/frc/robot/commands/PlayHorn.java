package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DoubleSolinoidSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PlayHorn extends Command {
    private final DoubleSolinoidSubsystem horn;
    private final Timer fireTimer = new Timer();

    public PlayHorn(DoubleSolinoidSubsystem horn) {
        this.horn = horn;
        addRequirements(horn);
    }

    @Override
    public void initialize() {
        fireTimer.start();
        System.out.println("fireing " + !horn.getSafety());
        if (!horn.getSafety()) {
            horn.setSolinoidState(Value.kForward);
        }
    }

    @Override
    public boolean isFinished() {
        return fireTimer.get() >= 0.25;
    }

    @Override
    public void end(boolean interrupted) {
        horn.getSolenoid().set(Value.kOff);
        fireTimer.stop();
        fireTimer.reset();
    }
}