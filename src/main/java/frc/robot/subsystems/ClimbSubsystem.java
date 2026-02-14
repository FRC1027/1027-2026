package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    // Primary climb motor controller.
    private final TalonFX climbMotor1;

    // Reused control request to avoid per-loop allocations.
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    /**
     * Initializes the climb subsystem by configuring the climb motor controller with
     * Motion Magic settings for repeatable position moves.
     */
    public ClimbSubsystem(){
        // Initialize the climb motor using configured CAN ID.
        climbMotor1 = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID);

        // Configure Motion Magic and closed-loop gains.
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID and Feedforward
        config.Slot0.kP = ClimbConstants.kP;
        config.Slot0.kI = ClimbConstants.kI;
        config.Slot0.kD = ClimbConstants.kD;
        config.Slot0.kV = ClimbConstants.kV;
        config.Slot0.kS = ClimbConstants.kS;
        config.Slot0.kG = ClimbConstants.kG;
        // config.Slot0.GravityType = GravityTypeValue.Elevator_Static; // Configure if using gravity compensation types

        // Motion Magic speed, acceleration, and jerk contrainsts
        config.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.MM_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ClimbConstants.MM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = ClimbConstants.MM_JERK;

        // Soft Limits
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.SOFT_LIMIT_FORWARD;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.SOFT_LIMIT_REVERSE;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // Current Limits
        config.CurrentLimits.StatorCurrentLimit = ClimbConstants.CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Feedback.SensorToMechanismRatio = ClimbConstants.CLIMB_SENSOR_TO_MECHANISM_RATIO;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climbMotor1.getConfigurator().apply(config);
        zeroClimbPosition();
    }

    /**
     * Command the climb to an absolute mechanism position in rotations using Motion Magic.
     *
     * @param targetRotations mechanism rotations from zeroed reference
     */
    public void setClimbPositionRotations(double targetRotations) {
        climbMotor1.setControl(motionMagicRequest.withPosition(targetRotations));
    }

    /**
     * Move the climb to the retracted preset.
     */
    public void retractClimb() {
        setClimbPositionRotations(ClimbConstants.CLIMB_RETRACTED_ROTATIONS);
    }

    /**
     * Move the climb to the extended preset.
     */
    public void extendClimb() {
        setClimbPositionRotations(ClimbConstants.CLIMB_EXTENDED_ROTATIONS);
    }

    /**
     * @return current mechanism position in rotations
     */
    public double getClimbPositionRotations() {
        return climbMotor1.getPosition().getValueAsDouble();
    }

    /**
     * @return current mechanism position in inches
     */
    public double getClimbPositionInches() {
        return getClimbPositionRotations() * ClimbConstants.CLIMB_INCHES_PER_MOTOR_ROTATION;
    }

    /**
     * Command the climb to an absolute mechanism position in inches.
     * 
     * @param targetInches target position in inches
     */
    public void setClimbPositionInches(double targetInches) {
        double targetRotations = targetInches / ClimbConstants.CLIMB_INCHES_PER_MOTOR_ROTATION;
        setClimbPositionRotations(targetRotations);
    }

    /**
     * Zero the climb position sensor.
     */
    public void zeroClimbPosition() {
        climbMotor1.setPosition(0.0);
    }
}
