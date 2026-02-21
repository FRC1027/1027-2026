package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class LockWheelsCommand extends Command{
    public LockWheelsCommand() {

    }

    @Override
    public void initialize() {
        // Runs once when the command starts
    }

    @Override
    public void execute() {
        // Runs repeatedly while the command is active (approx. every 20ms)
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when the command finishes or is interrupted
    }

    @Override
    public boolean isFinished() {
        // This command never finishes on its own; it will run until interrupted.
        return false;
    }
}
