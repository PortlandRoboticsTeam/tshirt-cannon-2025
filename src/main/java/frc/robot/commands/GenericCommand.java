package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class GenericCommand extends Command{
  Runnable onCommand = null;
  Runnable whileCommand = null;
  Runnable stopCommand = null;
  BooleanSupplier isDone = null;

  public GenericCommand(){}
  public GenericCommand(Runnable _whileCommand){
    whileCommand = _whileCommand;
  }
  public GenericCommand(Runnable _onCommand, Runnable _whileCommand, Runnable _stopCommand) {
    onCommand = _onCommand;
    whileCommand = _whileCommand;
    stopCommand = _stopCommand;
  }
  public GenericCommand(Runnable _onCommand, BooleanSupplier _isDone){
    onCommand = _onCommand;
    isDone = _isDone;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(onCommand!=null)
        onCommand.run();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(whileCommand!=null)
        whileCommand.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(stopCommand!=null)
        stopCommand.run();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isDone!=null)
      return isDone.getAsBoolean();
    return false;
  }
}