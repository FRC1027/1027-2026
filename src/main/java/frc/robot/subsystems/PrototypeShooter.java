package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveTowardTagCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utils;

public class PrototypeShooter extends SubsystemBase {
    // Minimum distance to the target (in inches)
    public static final int MINUMUM_DISTANCE = 90;

    // Maximum distance to the target (in inches)
    public static final int MAXIMUM_DISTANCE = 250;

    // Create the TalonFX motor controller for the shooter
    private TalonFX shooterMotor;

    public PrototypeShooter() {
        // Initialize the motor controller with a specific CAN ID
        shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID);

        // Configure the motor controller settings
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.15; // Proportional gain for velocity control (tuning may be required)
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterMotor.getConfigurator().apply(config);
    }

    /**
     * This method allows the driver to control the shooter motor manually.
     * 
     * Left Trigger: Shoots the object out (Outtake).
     * Right Trigger: Sucks the object in (Intake).
     * No Trigger: Stops the motor.
     */
    public Command manualShoot(){
        double rightTrigger = RobotContainer.driverXbox.getRightTriggerAxis();
        double leftTrigger = -RobotContainer.driverXbox.getLeftTriggerAxis();

        if (rightTrigger > 0.1){
            return run(() -> {
                setShooterSpeed(Utils.deadbandReturn((rightTrigger / 4), 0.1));
            });
        } else if (leftTrigger > 0.1){
            return run(() -> {
                setShooterSpeed(Utils.deadbandReturn((leftTrigger / 4), 0.1));
            });
        } else {
            return runOnce(() -> {
                setShooterSpeed(0);
            });
        }
    }

    // Command that aligns to the tag and then shoots
    public Command shootAlign(SwerveSubsystem drivebase) {
        double fid = LimelightHelpers.getFiducialID("limelight");

        if (fid == 4 || fid == 5 || fid == 25 || fid == 26) {
            return new DriveTowardTagCommand(drivebase, 0.0, 2.0)
            .andThen(shoot());
        } else {
            return Commands.none();
        }
    }

    /**
     * Runs the intake forward at a fixed speed for a certian period of time, then stops.
     *
     * @return command sequence for timed intake
     */
    public Command shoot() {
        return run(() -> {
            setShooterRPM();
        })//.withTimeout(ShooterConstants.SHOOTER_TIME)
        .andThen(runOnce(() -> shooterMotor.setControl(new VelocityVoltage(0))));
    }

    /**
     * Runs the intake in reverse at a fixed speed for 2 seconds, then stops.
     *
     * @return command sequence for timed outtake
     */
    public Command outtake() {
        return run(() -> {
            setShooterSpeed(-ShooterConstants.SHOOTER_POWER); // Reverse shooter for outtake
        })//.withTimeout(ShooterConstants.SHOOTER_TIME)
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