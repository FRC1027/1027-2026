package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.HopperConstants;

/**
 * Subsystem that controls hopper motors used to enlarge the holding space for the balls.
 * Motor 1 acts as leader; Motor 2 follows Motor 1.
 */
public class HopperSubsystem extends SubsystemBase {
    // Primary hopper motor controller (leader).
    private final TalonFX hopperMotor1;

    // Secondary hopper motor controller (follower).
    private final TalonFX hopperMotor2;

    // Boolean to indicate whether the hopper is currently expanded, for use in state management.
    private boolean hopperEnlarged;

    /**
     * Creates the hopper subsystem, configures both TalonFX motors, and
     * sets up the follower relationship for the second motor.
     */
    public HopperSubsystem() {
        // Default to not enlarged state on initialization; the hopper starts in its normal configuration.
        hopperEnlarged = false;

        // Initialize hopper motors using configured CAN IDs.
        hopperMotor1 = new TalonFX(HopperConstants.HOPPER_MOTOR_ID1);
        hopperMotor2 = new TalonFX(HopperConstants.HOPPER_MOTOR_ID2);

        // Configure velocity-loop gains and neutral mode for consistent behavior.
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.15; // Proportional gain for velocity control (tuning may be required).
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Apply the same configuration to both hopper motors.
        hopperMotor1.getConfigurator().apply(config);
        hopperMotor2.getConfigurator().apply(config);

        // Set motor 2 to follow motor 1 with opposite direction to match layout.
        hopperMotor2.setControl(new Follower(HopperConstants.HOPPER_MOTOR_ID1, MotorAlignmentValue.Opposed)); // Check if it should be Opposed or Aligned
    }

    /**
     * Manual hopper control using driver bumpers:
     * right bumper feeds forward, left bumper reverses, neither stops.
     */
    public Command manualHopperControl() {
        return run(() -> {
            Trigger rightBumper = RobotContainer.mechXbox.rightBumper();
            Trigger leftBumper = RobotContainer.mechXbox.leftBumper();

            if (rightBumper.getAsBoolean()) {
                setHopperSpeed(0.1027); // Set speed to 50% when right bumper is pressed
            } else if (leftBumper.getAsBoolean()) {
                setHopperSpeed(-0.1027); // Set speed to -50% when left bumper is pressed
            } else {
                setHopperSpeed(0.0); // Stop the hopper motor when neither bumper is pressed
            }
        });
    }

    /**
     * Enlarges or retracts the hopper based on its current state. If the hopper is not enlarged, 
     * it will run the motors to enlarge it; if it is already enlarged, it will run the motors in 
     * reverse to retract it.
     */
    public Command hopperEnlarger2000Command() {
        return Commands.defer(() -> {
            if (!hopperEnlarged) {
            // Example: run the hopper at 10% speed for 2 seconds to fully enlarge (tuning is required)
                return run(() -> setHopperSpeed(0.1))
                        .withTimeout(2.0) // Time how long it takes to fully extend the hopper with a known speed
                        .finallyDo(() -> {
                            setHopperSpeed(0.0);
                            hopperEnlarged = true;
                        });
            } else {
            // Example: run the hopper at 10% speed for 2 seconds to fully enlarge (tuning is required)
                return run(() -> setHopperSpeed(-0.1))
                        .withTimeout(2.0) // Time how long it takes to fully retract the hopper with a known speed
                        .finallyDo(() -> {
                            setHopperSpeed(0.0);
                            hopperEnlarged = false;
                        });
            }
        }, java.util.Set.of(this));
    }

    /**
     * Sets hopper leader motor output in the [-1, 1] range.
     *
     * @param speed motor output percent (sign controls direction)
     */
    public void setHopperSpeed(double speed) {
        hopperMotor1.set(speed); // Set the speed of the hopper motor
    }

    /**
     * Returns the current state of the hopper (enlarged or not). This is used for state management in commands and
     * other subsystems.
     * 
     * @return true if the hopper is enlarged, false otherwise
     */
    public boolean getHopperEnlarged() {
        return hopperEnlarged;
    }
}
