package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.RobotContainer;
import frc.robot.util.Constants.HopperConstants;

import java.util.Set;

/**
 * Subsystem that controls hopper motors used to enlarge the holding space for the balls.
 * Motor 1 acts as leader; Motor 2 follows Motor 1.
 */
public class HopperSubsystem extends SubsystemBase {
    // Primary hopper motor controller.
    private final TalonSRX hopperMotor1; //left

    // Secondary hopper motor controller.
    private final TalonSRX hopperMotor2; //right

    // Boolean to indicate whether the hopper is currently expanded, for use in state management.
    private boolean hopperEnlarged;

    /**
     * Creates the hopper subsystem, configures both TalonSRX motors, and
     * sets up the follower relationship for the second motor.
     */
    public HopperSubsystem() {
        // Default to not enlarged state on initialization; the hopper starts in its normal configuration.
        hopperEnlarged = false;

        // Initialize hopper motors using configured CAN IDs.
        hopperMotor1 = new TalonSRX(HopperConstants.HOPPER_MOTOR_ID1);
        hopperMotor2 = new TalonSRX(HopperConstants.HOPPER_MOTOR_ID2);
    }

    /**
     * Manual hopper control using driver bumpers:
     * right bumper feeds forward, left bumper reverses, neither stops.
     * 
     * KNOWN ISSUE: This command does not update the hopperEnlarged state variable.
     */
    public Command manualHopperControl() {
        return runEnd(() -> {
            Trigger rightBumper = RobotContainer.mechXbox.rightBumper();
            Trigger leftBumper = RobotContainer.mechXbox.leftBumper();

            if (rightBumper.getAsBoolean()) {
                setHopperSpeed(0.7); // Run hopper forward while right bumper is held.
            } else if (leftBumper.getAsBoolean()) {
                setHopperSpeed(-0.7); // Run hopper in reverse while left bumper is held.
            } else {
                setHopperSpeed(0.0); // Stop when neither bumper is pressed.
            }
        }, () -> setHopperSpeed(.0)); // Ensure motors stop when the command ends
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
        }, Set.of(this));
    }

    /**
     * Sets hopper leader motor output in the [-1, 1] range.
     *
     * @param speed motor output percent (sign controls direction)
     */
    public void setHopperSpeed(double speed) {
        hopperMotor1.set(TalonSRXControlMode.PercentOutput, speed); // Set the speed of the primary hopper motor
        hopperMotor2.set(TalonSRXControlMode.PercentOutput, -speed); // Set the speed of the secondary hopper motor
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
