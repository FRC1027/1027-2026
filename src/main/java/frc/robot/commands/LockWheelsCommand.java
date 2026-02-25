package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class LockWheelsCommand extends Command {
    private final SwerveSubsystem swerve;

    public LockWheelsCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Runs once when the command starts
        SmartDashboard.putString("Wheel Lock Status", "Wheels are Now Locked");
    }

    @Override
    public void execute() {
        // Runs repeatedly while the command is active (approx. every 20ms)
        swerve.lock();

        SmartDashboard.putString("Wheel Lock Status", "Wheels are Locked");
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when the command finishes or is interrupted
        SmartDashboard.putString("Wheel Lock Status", "Wheels are Now Unlocked");
    }

    @Override
    public boolean isFinished() {
        // This command never finishes on its own; it will run until interrupted.
        return false;
    }
}
