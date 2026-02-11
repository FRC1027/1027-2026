package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase
{
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
        config.Slot0.kP = ClimbConstants.CLIMB_KP;
        config.Slot0.kI = ClimbConstants.CLIMB_KI;
        config.Slot0.kD = ClimbConstants.CLIMB_KD;
        config.Slot0.kV = ClimbConstants.CLIMB_KV;
        config.Slot0.kS = ClimbConstants.CLIMB_KS;
        config.Slot0.kG = ClimbConstants.CLIMB_KG;

        config.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.CLIMB_CRUISE_VELOCITY_RPS;
        config.MotionMagic.MotionMagicAcceleration = ClimbConstants.CLIMB_ACCELERATION_RPS2;
        config.MotionMagic.MotionMagicJerk = ClimbConstants.CLIMB_JERK_RPS3;

        config.Feedback.SensorToMechanismRatio = ClimbConstants.CLIMB_SENSOR_TO_MECHANISM_RATIO;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climbMotor1.getConfigurator().apply(config);
        zeroClimbPosition();
    }

    /**
     * Manual open-loop climb output in range [-1, 1].
     *
     * @param power motor percent output
     */
    public void setClimbPower(double power){
        climbMotor1.set(MathUtil.clamp(power, -1.0, 1.0));
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
     * Command a relative climb move in rotations from current position.
     *
     * @param deltaRotations signed mechanism rotation offset
     */
    public void moveClimbByRotations(double deltaRotations) {
        setClimbPositionRotations(getClimbPositionRotations() + deltaRotations);
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
     * Zero the climb position sensor.
     */
    public void zeroClimbPosition() {
        climbMotor1.setPosition(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb/PositionRotations", getClimbPositionRotations());
    }
}
