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

import frc.robot.util.Constants.IndexerConstants;

/**
 * Subsystem that controls the indexer motor used to feed game pieces into the shooter.
 */
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

        // Configure the indexer motor using safe parameter reset and persistent parameter storage.
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Returns a command that allows manual control of the indexer motor using the right Y-axis of the mechXbox controller.
     * 
     * @return command that allows manual control of the indexer motor.
     */
    public Command manualIndexerCommand() {
        return runEnd(
            () -> setIndexerSpeed(1),
            () -> setIndexerSpeed(.0)
        );
    }

    /**
     * Returns the current feed command used by shooter routines.
     * 
     * Constructs a delayed command that starts the indexer and allows the shooter to spin up before
     * feeding, while ensuring that the indexer is stopped when the command ends or is interrupted.
     * 
     * @return command that leaves the indexer stopped and forces zero output on end/interruption
     */
    public Command runIndexerCommand() {
        return Commands.sequence(
            Commands.waitSeconds(3.0), // Delays the start of the indexer by a set time interval
            run(() -> setIndexerSpeed(1.0))
        ).finallyDo(interrupted -> setIndexerSpeed(0.0)); // Ensure indexer is stopped when this command ends or is interrupted.
    }

    /**
     * Run the indexer in reverse for a short interval of time to release the intake/hopper mechanism
     * 
     * @return a command that runs the indexer motor in reverse for 3 seconds, then stops
     */
    public Command releaseIntakeCommand() {
        return run(() -> setIndexerSpeed(-0.5))
            .withTimeout(3)
            .finallyDo(interrupted -> setIndexerSpeed(0));
    }

    /**
     * Sets the speed of the indexer motor.
     * @param speed The speed to set the motor to (between -1.0 and 1.0).
     */
    public void setIndexerSpeed(double speed) {
        indexerMotor.set(speed);
    }
}
