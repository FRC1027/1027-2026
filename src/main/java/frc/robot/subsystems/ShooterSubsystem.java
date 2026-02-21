package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.commands.DriveTowardTargetCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.Constants.ObjectRecognitionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utils;

public class ShooterSubsystem extends SubsystemBase {
    // Limelight NetworkTable used to fetch target 3D pose data.
    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable(ObjectRecognitionConstants.LIMELIGHT_NAME);

    // Primary shooter motor controller (leader).
    private TalonFX shooterMotor1;

    // Secondary shooter motor controller (follower).
    private TalonFX shooterMotor2;

    /**
     * Creates the shooter subsystem, configures TalonFX control gains, and
     * sets up follower behavior and dashboard tuning entries.
     */
    public ShooterSubsystem() {
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
        shooterMotor2.getConfigurator().apply(config);

        // Set the second motor to follow the first motor with opposite direction to match motor layout.
        shooterMotor2.setControl(new Follower(ShooterConstants.SHOOTER_MOTOR_ID1, MotorAlignmentValue.Opposed)); // Check if it should be Opposed or Aligned

        // Publish the radius efficiency to SmartDashboard so it can be tuned live.
        SmartDashboard.putNumber("Shooter/RadiusEfficiency", ShooterConstants.DEFAULT_RADIUS_EFFICIENCY);
    }

    /**
     * Calculates the effective radius of the shooter wheel by applying an efficiency factor 
     * to account for real-world conditions. Allows for tuning the efficiency factor via the 
     * SmartDashboard to improve accuracy of RPS calculations.
     * 
     * @return Effective radius of the shooter wheel in meters.
     */
    private double getEffectiveRadius() {
        double efficiency = SmartDashboard.getNumber("Shooter/RadiusEfficiency", ShooterConstants.DEFAULT_RADIUS_EFFICIENCY);

        // Clamp the efficiency to a reasonable range to avoid unrealistic tuning values.
        efficiency = MathUtil.clamp(efficiency, 0.5, 1.0);
        return ShooterConstants.SHOOTER_WHEEL_RADIUS * efficiency;
    }

    /**
     * Calculates the required launch RPS (revolutions per second) to hit the target
     * based on the current distance from the bumper to the target.
     * 
     * @return Required launch RPS to hit the target, or NaN if the shot is not feasible.
     */
    public double calculateWheelRPS() {
        // Calculate the distance from the bumper to the target tag using Limelight data.
        double bumperToTagDistance = Utils.calculateDistanceToTarget(limelight);

        // Return NaN if the distance data is missing or invalid.
        if (!Double.isFinite(bumperToTagDistance)) {
            return Double.NaN;
        }

        // If the target is too high relative to the launch angle, the projectile motion equation would be invalid.
        if (bumperToTagDistance * Math.tan(ShooterConstants.SHOOTER_ANGLE) <= ShooterConstants.HEIGHT_DIFFERENCE) {
            return Double.NaN;
        }

        // Clamp the distance to reduce sensitivity to outliers or bad measurements.
        bumperToTagDistance = MathUtil.clamp(bumperToTagDistance, ShooterConstants.MINIMUM_DISTANCE, ShooterConstants.MAXIMUM_DISTANCE);

        // Calculate required launch velocity using projectile motion (ignoring air resistance).
        double velocity = Math.sqrt((ShooterConstants.GRAVITY_CONSTANT * bumperToTagDistance * bumperToTagDistance) / 
                                    (2 * Math.cos(ShooterConstants.SHOOTER_ANGLE) * Math.cos(ShooterConstants.SHOOTER_ANGLE) * 
                                    (bumperToTagDistance * Math.tan(ShooterConstants.SHOOTER_ANGLE) - ShooterConstants.HEIGHT_DIFFERENCE)));

        // Convert linear velocity to wheel RPS using the effective radius (includes efficiency factor).
        double rps = velocity / (2 * Math.PI * getEffectiveRadius());

        // Return the calculated wheel speed in revolutions per second.
        return rps;
    }

    /**
     * This method allows the driver to control the shooter motor manually.
     * 
     * Left Trigger: Intakes (negative speed).
     * Right Trigger: Shoots out (positive speed).
     * No Trigger: Stops the motor.
     */
    public Command manualShoot() {
        return run(() -> {
            double rightTrigger = Utils.deadbandReturn(RobotContainer.driverXbox.getRightTriggerAxis(), 0.1);
            double leftTrigger = -Utils.deadbandReturn(RobotContainer.driverXbox.getLeftTriggerAxis(), 0.1);

            if (rightTrigger > 0.0){
                // Scale trigger input after applying deadband for smooth manual shooting control.
                setShooterSpeed(rightTrigger / 4);
            } else if (leftTrigger < 0.0){
                // Scale trigger input after applying deadband for smooth manual intake control.
                setShooterSpeed(leftTrigger / 4);
            } else {
                // No trigger input: stop the shooter motor.
                setShooterSpeed(0.0);
            }
        });
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
            double fid = LimelightHelpers.getFiducialID("limelight");

            if (fid == 4 || fid == 9 || fid == 10 || fid == 25 || fid == 26) {
                // Align with the tag at a fixed rotation speed, then run the shooter command.
                return new DriveTowardTargetCommand(drivebase, 0.0, 2.0)
                .andThen(shoot());
            }
            // If no valid tag is seen, return a "do-nothing" command to avoid unintended motion.
            return Commands.none();
        }, java.util.Collections.singleton(this));
    }

    /**
     * Runs the intake forward at a fixed speed for a certian period of time, then stops.
     *
     * @return command that spins the shooter using Limelight-based RPS until interrupted
     */
    public Command shoot() {
        return runEnd(
            this::setShooterRPS,
            () -> shooterMotor1.setControl(new VelocityVoltage(0))
        );
    }

    /**
     * Runs the intake in reverse at a fixed speed for 2 seconds, then stops.
     * 
     * @return command that reverses the shooter for a short, timed outtake
     */
    public Command outtake() {
        return run(() -> {
            setShooterSpeed(-ShooterConstants.SHOOTER_POWER); // Reverse shooter for outtake
        }).withTimeout(2.0) // Run for 2 seconds
          .andThen(runOnce(() -> shooterMotor1.setControl(new VelocityVoltage(0)))); 
    }

    /**
     * Sets the shooter motor to a RPS calculated via relevant data from Limelight.
     */
    public void setShooterRPS() {
        double wheelRPS = calculateWheelRPS();

        // If the calculated RPS is not valid, stop the motor to avoid unintentional behavior.
        if (!Double.isFinite(wheelRPS)) {
            shooterMotor1.setControl(new VelocityVoltage(0));
            return;
        }

        // Convert wheel RPS to motor RPS using the configured gear ratio.
        double motorRPS = wheelRPS * ShooterConstants.GEAR_RATIO;
        shooterMotor1.setControl(new VelocityVoltage(motorRPS));
    }

    /**
     * Sets the shooter motor speed for manual control. Positive speed shoots out,
     * negative speed intakes in, and zero stops the motor.
     * 
     * @param speed motor output in the [-1, 1] range (sign controls direction)
     */
    public void setShooterSpeed(double speed) {
        shooterMotor1.set(speed);
    }
}
