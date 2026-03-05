package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.util.Constants.IndexerConstants;
import frc.robot.util.Utils;

public class IndexerSubsystem extends SubsystemBase {
    
    // Indexer motor
    private final SparkMax indexerMotor;

    // Motor configuration for both intake motors
    public static final SparkMaxConfig indexerConfig = new SparkMaxConfig();

    static {
        indexerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
    }
    
    /**
     * Constructor for the IndexerSubsystem. Initializes the indexer motor and applies the shared configuration.
     */
    @SuppressWarnings("removal") // Suppress warnings about deprecated ResetMode and PersistMode usage in SparkMax configuration.
    public IndexerSubsystem() {
        // Initialize the indexer motor using configured CAN ID.
        indexerMotor = new SparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

        // Configure the indexer motor with the shared configuration, using safe parameter reset and persistent parameter storage.
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Returns a command that allows manual control of the indexer motor using the right Y-axis of the mechXbox controller.
     * 
     * @return command that allows manual control of the indexer motor.
     */
    public Command manualIndexerCommand() {
        return run(() -> {
            // Get the right joystick X-axis value from the mechXbox controller to control the indexer motor speed.
            double yAxisJoystickValue = Utils.deadbandReturn(RobotContainer.mechXbox.getRightY(), 0.1);

            if (yAxisJoystickValue > 0) {
                // If the joystick is pushed to the right, set the indexer motor speed to the joystick value (positive).
                setIndexerSpeed(yAxisJoystickValue / 4);
            } else if (yAxisJoystickValue < 0) {
                // If the joystick is pushed to the left, set the indexer motor speed to the joystick value (negative).
                setIndexerSpeed(yAxisJoystickValue / 4);
            } else {
                // If the joystick is in the deadband (near center), stop the indexer motor.
                setIndexerSpeed(0);
            }
        });
    }

    /**
     * Runs the indexer motor at a fixed speed after a delay, then stops.
     * 
     * @return command that waits for 3 seconds before running the indexer motor at half speed,
     * then stops the indexer motor when the command ends or is interrupted.
     */
    public Command runIndexerCommand() {
        return runEnd(
            () -> Commands.waitSeconds(3.0) // Wait for 3 seconds before starting the indexer motor to allow time for the shooter to spin up.
                    .andThen(() -> setIndexerSpeed(0.5)), // Then run the indexer motor at half speed when the command is active.
            () -> setIndexerSpeed(0.0)    // Stop the indexer motor when the command ends or is interrupted.
        );
    }

    /**
     * Sets the speed of the indexer motor.
     * @param speed The speed to set the motor to (between -1.0 and 1.0).
     */
    public void setIndexerSpeed(double speed) {
        indexerMotor.set(speed);
    }
}
