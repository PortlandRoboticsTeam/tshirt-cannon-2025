package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SolinoidSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class FireCannon extends Command {
    private final SolinoidSubsystem tshirtCannon;
    private final Timer fireTimer = new Timer();

    public FireCannon(SolinoidSubsystem tshirtCannon) {
        this.tshirtCannon = tshirtCannon;
        addRequirements(tshirtCannon);
    }

    @Override
    public void initialize() {
        fireTimer.start();
        System.out.println("fireing " + !tshirtCannon.getSafety());
        if (!tshirtCannon.getSafety()) {
            tshirtCannon.setSolinoidState(true);
        }
    }

    @Override
    public boolean isFinished() {
        return fireTimer.get() >= 0.25;
    }

    @Override
    public void end(boolean interrupted) {
        tshirtCannon.getSolenoid().set(false);
        fireTimer.stop();
        fireTimer.reset();
    }
}