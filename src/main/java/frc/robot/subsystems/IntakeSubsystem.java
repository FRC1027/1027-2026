package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.Utils;

public class IntakeSubsystem extends SubsystemBase {
    // Reference to the hopper subsystem for potential coordination between intake and hopper operations.
    private HopperSubsystem hopperSubsystem = new HopperSubsystem();

    // Intake motor
    private SparkMax intakeMotor;

    // Shared motor configuration for all intake instances
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
        intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
    }

    @SuppressWarnings("removal") // Suppress warnings about deprecated ResetMode and PersistMode usage in SparkMax configuration
    public IntakeSubsystem() {
        // Initialize the intake motor using configured CAN ID.
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

        // Configure the intake motor with the shared configuration, using safe parameter reset and persistent parameter storage.
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Creates a command that allows manual control of the intake motor using the Xbox controller triggers.
     * Right Trigger: Intakes (positive speed).
     * Left Trigger: Outtakes (negative speed).
     * No Trigger: Stops the motor.
     */
    public Command manualIntakeCommand() {
        return run(() -> {
            double rightTrigger = Utils.deadbandReturn(RobotContainer.mechXbox.getRightTriggerAxis(), 0.1);
            double leftTrigger = -Utils.deadbandReturn(RobotContainer.mechXbox.getLeftTriggerAxis(), 0.1);

            if (rightTrigger > 0.0){
                // Scale trigger input after applying deadband for smooth manual intake control.
                setIntakeSpeed(rightTrigger / 4);
            } else if (leftTrigger < 0.0){
                // Scale trigger input after applying deadband for smooth manual outake control.
                setIntakeSpeed(leftTrigger / 4);
            } else {
                // No trigger input: stop the motor.
                setIntakeSpeed(0.0);
            }
        });
    }

    /**
     * Creates a command that continuously runs the intake motor at a fixed speed when the hopper is enlarged,
     * and stops it when the hopper is not enlarged. This allows for automatic coordination between the intake
     * and hopper subsystems based on the hopper's state.
     */
    public Command continuousIntakeCommand() { //
        return run(() -> {
            if (hopperSubsystem.getHopperEnlarged()) {
                setIntakeSpeed(0.5); // Run intake at 50% speed when hopper is enlarged
            } else {
                setIntakeSpeed(0.0); // Stop intake when hopper is not enlarged
            }
        });
    }

    /**
     * Sets the speed of the intake motor.
     * @param speed The speed to set the motor to (between -1.0 and 1.0).
     */
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }
}
