package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.DriveTowardTargetCommand;
import frc.robot.commands.LockWheelsCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.Constants.ObjectRecognitionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utils;

import java.util.Set;

/**
 * Subsystem that controls the shooter flywheels and shot execution commands.
 */
public class ShooterSubsystem extends SubsystemBase {
    // private static final InterpolatingDoubleTreeMap shooterTableRPS = new InterpolatingDoubleTreeMap();

    // static {
    //     shooterTableRPS.put(0.5, 50.0); // distance (meters), RPS
    // }

    // private static final InterpolatingDoubleTreeMap shooterTableCoefficient = new InterpolatingDoubleTreeMap();

    // static {

    // }

    // Limelight NetworkTable used to fetch target 3D pose data.
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable(ObjectRecognitionConstants.LIMELIGHT_NAME);

    // Reference to the IndexerSubsystem to run the indexer command in parallel with shooting.
    private final IndexerSubsystem m_indexer;

    // Primary shooter motor controller (leader).
    private final TalonFX shooterMotor1;

    // Secondary shooter motor controller (follower).
    private final TalonFX shooterMotor2;

    // The follow object that controls the 2nd motor's behavior
    private final StrictFollower followerRequest = new StrictFollower(ShooterConstants.SHOOTER_MOTOR_ID1);

    /**
     * Creates the shooter subsystem, configures TalonFX control gains, and sets up
     * follower behavior and dashboard tuning entries.
     */
    public ShooterSubsystem(IndexerSubsystem m_indexer) {
        // Store the reference to the IndexerSubsystem for use in shooting commands.
        this.m_indexer = m_indexer;

        // Initialize the shooter motors using configured CAN IDs.
        shooterMotor1 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID1);
        shooterMotor2 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID2);

        // Configure velocity-loop gains and neutral mode for consistent flywheel behavior.
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.15; // Proportional gain for velocity control (tuning may be required).
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterMotor1.getConfigurator().apply(config);

        // Set Motor 2 to run physically opposed to Motor 1 on the shared shaft.
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterMotor2.getConfigurator().apply(config);

        // Publish the radius efficiency to SmartDashboard so it can be tuned live.
        SmartDashboard.putNumber("Shooter/VelocityEfficiency", ShooterConstants.VELOCITY_EFFICIENCY);

        // Publish a changable shooter to target distance value in the case that the AprilTag is not visible.
        SmartDashboard.putNumber("Shooter/TunableDistance", ShooterConstants.MINIMUM_DISTANCE);
    }

    /**
     * Calculates the effective velocity of the shooter wheel by applying an efficiency factor 
     * to account for real-world conditions. Allows for tuning the efficiency factor via the 
     * SmartDashboard to improve accuracy of RPS calculations.
     * 
     * @return Effective velocity of the shooter wheel in meters.
     */
    private double getAdjustedVelocity(double velocity) {
        double efficiency = SmartDashboard.getNumber("Shooter/VelocityEfficiency", ShooterConstants.VELOCITY_EFFICIENCY);

        return velocity * efficiency;
    }

    /**
     * Calculates the required launch RPS (revolutions per second) to hit the target
     * based on the current distance from the bumper to the target.
     *
     * @return Required launch RPS to hit the target, or NaN if the shot is not feasible.
     */
    public double calculateWheelRPS() {
        // Calculate the distance from the shooter to the target tag using Limelight data.
        double shooterToTag = Utils.calculateDistanceToTarget(limelight);

        // Return NaN if the distance data is missing or invalid.
        if (!Double.isFinite(shooterToTag)) {
            return Double.NaN;
        }

        // If the target is too high relative to the launch angle, the projectile motion equation would be invalid.
        if (shooterToTag * Math.tan(ShooterConstants.SHOOTER_ANGLE) <= ShooterConstants.HEIGHT_DIFFERENCE) {
            return Double.NaN;
        }

        // Calculate required launch velocity using projectile motion (ignoring air resistance).
        double velocity = Math.sqrt((ShooterConstants.GRAVITY_CONSTANT * shooterToTag * shooterToTag) / 
                                    (2 * Math.cos(ShooterConstants.SHOOTER_ANGLE) * Math.cos(ShooterConstants.SHOOTER_ANGLE) * 
                                    (shooterToTag * Math.tan(ShooterConstants.SHOOTER_ANGLE) - ShooterConstants.HEIGHT_DIFFERENCE)));

        double adjustedVelocity = getAdjustedVelocity(velocity);
        
        // Convert linear velocity to wheel RPS using the effective velocity value.
        double rps = adjustedVelocity / (2 * Math.PI * ShooterConstants.SHOOTER_WHEEL_RADIUS);

        // Return the calculated wheel speed in revolutions per second (Multiply by Compression Loss Compensation).
        return rps * 1.03;
    }

    /**
     * Aligns the robot to the target tag and shoots if the tag ID is valid.
     * Valid tag IDs are in front of Red Hub (9 or 10), in front of Blue Hub (25 or 26), or 4 for testing in the RCH.
     *
     * @param drivebase swerve subsystem used to drive toward the tag
     * @return command that aligns to the tag and then shoots, or no-op if tag is invalid
     */
    public Command shootAlign(SwerveSubsystem drivebase) {
        return Commands.defer(() -> {
            double fid = LimelightHelpers.getFiducialID(ObjectRecognitionConstants.LIMELIGHT_NAME);

            if (fid == 4 || fid == 9 || fid == 10 || fid == 25 || fid == 26) {
                return new DriveTowardTargetCommand(drivebase, 0.0, 2.0) // Aligns to the target tag using only rotational movement (max speed = 0)
                        // Once the alignment command finishes, run the shoot command while also locking the wheels to prevent movement during shooting.
                        .andThen(Commands.deadline(
                                shoot(), // End the command when the shoot command finishes (which is when the driver releases the trigger)
                                new LockWheelsCommand(drivebase).repeatedly()));
            }
            // If no valid tag is seen, return a "do-nothing" command to avoid unintended motion.
            return Commands.none();
        }, Set.of(this, drivebase));
    }

    /**
     * Runs the intake forward at a fixed speed for a certian period of time, then stops.
     *
     * @return command that spins the shooter using Limelight-based RPS until interrupted, in parallel with
     * the indexer that feeds balls into the shooter, then stops the shooter and indexer when the command ends.
     */
    public Command shoot() {
        final double[] targetRPS = new double[1];

        return Commands.sequence(
            // Step 1: calculate once
            Commands.runOnce(() -> targetRPS[0] = calculateWheelRPS()),

            // Step 2: run shooter using stored value
            Commands.deadline(
                runEnd(
                    () -> setShooterRPS(targetRPS[0]),
                    () -> {
                        shooterMotor1.setControl(new NeutralOut());
                        shooterMotor2.setControl(followerRequest);
                    }
                ),
                m_indexer.runIndexerCommand()
            )
        );
    }

    public Command shootBrake(SwerveSubsystem drivebase) {
        final double[] targetRPS = new double[1];

        return Commands.sequence(
            Commands.runOnce(() -> targetRPS[0] = calculateWheelRPS()),

            Commands.deadline(
                Commands.deadline(
                    runEnd(
                        () -> setShooterRPS(targetRPS[0]),
                        () -> {
                            shooterMotor1.setControl(new NeutralOut());
                            shooterMotor2.setControl(followerRequest);
                        }
                    ),
                    m_indexer.runIndexerCommand()
                ),
                new LockWheelsCommand(drivebase)
            )
        );
    }

    // Method used for shooter testing to run the indexer and shooter at full speed without Limelight distance calculation
    public Command fullSpeed(){
        return Commands.deadline(
            runEnd(
                () -> setShooterSpeed(1.0), 
                () -> {
                    shooterMotor1.setControl(new NeutralOut());
                    shooterMotor2.setControl(followerRequest);
                }
            ),
            m_indexer.runIndexerCommand());
    }

    public Command rightMotor(){
        return runEnd(
            () -> setShooterSpeed(0.5),
            () -> shooterMotor2.setControl(new NeutralOut())
        );
    }

    /**
     * Sets the shooter motor to a RPS calculated via relevant data from Limelight.
     */
    public void setShooterRPS(double wheelRPS) {
        // If the calculated RPS is not valid, stop the motor to avoid unintentional behavior.
        if (!Double.isFinite(wheelRPS)) {
            shooterMotor1.setControl(new NeutralOut());
            shooterMotor2.setControl(followerRequest);
            return;
        }

        // Convert wheel RPS to motor RPS using the configured gear ratio.
        double motorRPS = wheelRPS * ShooterConstants.GEAR_RATIO;
        shooterMotor1.setControl(new VelocityVoltage(motorRPS));
        shooterMotor2.setControl(followerRequest);
    }

    public void setShooterRPS() {
        setShooterRPS(calculateWheelRPS());
    }

    /**
     * Sets the shooter motor speed for manual control. Positive speed shoots out,
     * negative speed intakes in, and zero stops the motor.
     *
     * @param speed motor output in the [-1, 1] range (sign controls direction)
     */
    public void setShooterSpeed(double speed) {
        //shooterMotor1.set(speed);
        //shooterMotor2.setControl(followerRequest);
        shooterMotor2.set(speed);
    }
}
