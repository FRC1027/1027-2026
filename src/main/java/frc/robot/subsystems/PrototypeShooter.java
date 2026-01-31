package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveTowardTagCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.Utils;

public class PrototypeShooter extends SubsystemBase {
    // Minimum distance to the target (in inches)
    public static final int MINUMUM_DISTANCE = 90;

    // Maximum distance to the target (in inches)
    public static final int MAXIMUM_DISTANCE = 250;

    private TalonFX shooterMotor;

    public PrototypeShooter() {
        // Initialize the motor controller with a specific CAN ID
        shooterMotor = new TalonFX(23);
    }

    /**
     * This method runs automatically, over and over again (about 50 times a second).
     * 
     * It checks the controller triggers to see if the driver wants to shoot or intake.
     * 
     * Left Trigger: Shoots the note out (Outtake).
     * Right Trigger: Sucks the note in (Intake).
     * No Trigger: Stops the motor.
     */
    @Override
    public void periodic() {
        double rightTrigger = RobotContainer.driverXbox.getRightTriggerAxis();
        double leftTrigger = -RobotContainer.driverXbox.getLeftTriggerAxis();

        // Priority order: Outtake > Intake > Stop
        if (RobotContainer.driverXbox.getLeftTriggerAxis() > 0.1) {
            // Outtake (left trigger) if pressed beyond deadband
            setShooterSpeed(leftTrigger / 4);
        } else if (RobotContainer.driverXbox.getRightTriggerAxis() > 0.1) {
            // Intake (right trigger) if pressed beyond deadband
            setShooterSpeed(rightTrigger / 4);
        } else {
            // Neither trigger pressed → stop
            setShooterSpeed(0);
        }
    }

    /** Periodically called during simulation (currently unused). */
    @Override
    public void simulationPeriodic() {}

    // Command that aligns to the tag and then shoots
    public Command shootAlign(SwerveSubsystem drivebase) {
        return run(() -> {
            new DriveTowardTagCommand(drivebase);
        }).andThen(shoot());
    }

    /**
     * Runs the intake forward at a fixed speed for a certian period of time, then stops.
     *
     * @return command sequence for timed intake
     */
    public Command shoot() {
        return run(() -> {
            setShooterRPM();
            //setShooterSpeed(ShooterConstants.SHOOTER_POWER);
        })//.withTimeout(ShooterConstants.SHOOTER_TIME)
        .andThen(runOnce(() -> shooterMotor.set(0)));
    }

    /**
     * Runs the intake in reverse at a fixed speed for 2 seconds, then stops.
     *
     * @return command sequence for timed outtake
     */
    public Command outtake() {
        return run(() -> {
            setShooterSpeed(-ShooterConstants.SHOOTER_POWER); // Reverse shooter for outtake
        }).withTimeout(ShooterConstants.SHOOTER_TIME)
        .andThen(runOnce(() -> shooterMotor.set(0)));
    }

    // Sets the shooter motor RPM to the target value
    public void setShooterRPM() {
        shooterMotor.setControl(new VelocityVoltage(ShooterConstants.SHOOTER_TARGET_RPS));
    }

    // Sets the shooter motor speed with deadband applied
    public void setShooterSpeed(double speed) {
        shooterMotor.set(speed);
    }
}