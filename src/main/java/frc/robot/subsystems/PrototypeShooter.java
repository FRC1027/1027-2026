package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveTowardTagCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.Constants.RobotProperties;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utils;

public class PrototypeShooter extends SubsystemBase {
    // Minimum distance to the target (in meters)
    private static final double MINIMUM_DISTANCE = 90 * 0.0254; // 90 inches to meters

    // Maximum distance to the target (in meters)
    private static final double MAXIMUM_DISTANCE = 250 * 0.0254; // 250 inches to meters

    // Gravity constant (m/s^2)
    private static final double GRAVITY_CONSTANT = 9.81;

    // Shooter wheel radius (meters)
    private static final double SHOOTER_WHEEL_RADIUS = 2.25 * 0.0254; // 2.25 inches to meters

    // Fudge factor to account for real-world conditions ( between 0 < k < 1; determine experimentally)
    private static final double radiusEfficiency = 0.85; // ADJUST THIS VALUE BASED ON TESTING

    // Find the effective radius to account for ball compression, slip, and friction (meters)
    private static final double EFFECTIVE_RADIUS = SHOOTER_WHEEL_RADIUS * radiusEfficiency; // Multiple by fudge factor

    // Shooter angle (radians)
    private static final double SHOOTER_ANGLE = Math.toRadians(45.0); // Convert 45 degrees to radians

    // Shooter height (meters)
    private static final double SHOOTER_HEIGHT = 27 * 0.0254; // 27 inches to meters

    // Target height (meters)
    private static final double GOAL_HEIGHT = 72 * 0.0254; // 72 inches to meters

    // Height difference between target and shooter (meters)
    private static final double HEIGHT_DIFFERENCE = GOAL_HEIGHT - SHOOTER_HEIGHT;

    // NetworkTable for Limelight
    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    // Create the TalonFX motor controller for the shooter
    private TalonFX shooterMotor;


    /**
     * Constructor for the PrototypeShooter subsystem.
     */
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
     * Calculates the distance from the bumper to the target tag using Limelight data.
     * 
     * @return Distance from bumper to target tag in meters
     */
    private double getBumperToTagDistance() {
        // Get the pose of the target relative to the camera
        double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);

        if (pose.length < 3) {
            return Double.NaN;
        }

        double tx = pose[0]; // Horizontal offset (left/right) in meters
        double ty = pose[1]; // Vertical offset (up/down) in meters
        double tz = pose[2]; // Forward distance (depth) in meters

        // Calculate the distance from the camera to the tag in meters
        double cameraToTag = Math.sqrt(tx * tx + ty * ty + tz * tz);

        // Calculate and return the distance from the bumper to the tag
        return Math.max(0.0, cameraToTag - RobotProperties.CAM_TO_BUMPER_DISTANCE);
    }

    /**
     * Calculates the required launch RPS (revolutions per second) to hit the target
     * based on the current distance from the bumper to the target.
     * 
     * @return Required launch RPS to hit the target
     */
    public double calculateWheelRPS() {
        double bumperToTagDistance = getBumperToTagDistance();

        // Return NaN if distance is not valid
        if (!Double.isFinite(bumperToTagDistance)) {
            return Double.NaN;
        }

        // Check if the target is reachable with the given shooter angle
        if (bumperToTagDistance * Math.tan(SHOOTER_ANGLE) <= HEIGHT_DIFFERENCE) {
            return Double.NaN;
        }

        // Clamp the distance to within the min and max limits
        bumperToTagDistance = MathUtil.clamp(bumperToTagDistance, MINIMUM_DISTANCE, MAXIMUM_DISTANCE);

        // Calculate the launch velocity using the projectile motion formula
        double velocity = Math.sqrt((GRAVITY_CONSTANT * bumperToTagDistance * bumperToTagDistance) / 
                                    (2 * Math.cos(SHOOTER_ANGLE) * Math.cos(SHOOTER_ANGLE) * 
                                    (bumperToTagDistance * Math.tan(SHOOTER_ANGLE) - HEIGHT_DIFFERENCE)));

        // Convert velocity to RPS (revolutions per second)
        double rps = velocity / (2 * Math.PI * EFFECTIVE_RADIUS); // Include fudge factor to adjust for real-world conditions

        return rps; // Return the calculated RPS
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
        } else if (leftTrigger < -0.1){
            return run(() -> {
                setShooterSpeed(Utils.deadbandReturn((leftTrigger / 4), 0.1));
            });
        } else {
            return runOnce(() -> {
                setShooterSpeed(0);
            });
        }
    }

    /**
     * Aligns the robot to the target tag and shoots if the tag ID is valid.
     * Valid tag IDs are in front of Red Hub (9 or 10), in front of Blue Hub (25 or 26), or 4 for testing in the RCH.
     * 
     * @param drivebase
     * @return
     */
    public Command shootAlign(SwerveSubsystem drivebase) {
        double fid = LimelightHelpers.getFiducialID("limelight");

        if (fid == 4 || fid == 9 || fid == 10 || fid == 25 || fid == 26) {
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
        return startEnd(
            this::setShooterRPS,
            () -> shooterMotor.setControl(new VelocityVoltage(0))
        );
    }

    /**
     * Runs the intake in reverse at a fixed speed for 2 seconds, then stops.
     * 
     * @return command sequence for timed outtake
     */
    public Command outtake() {
        return run(() -> {
            setShooterSpeed(-ShooterConstants.SHOOTER_POWER); // Reverse shooter for outtake
        }).andThen(runOnce(() -> shooterMotor.set(0)));
    }

    /**
     * Sets the shooter motor to a RPS calculated via relevant data from Limelight.
     */
    public void setShooterRPS() {
        double wheelRPS = calculateWheelRPS();

        if (!Double.isFinite(wheelRPS)) {
            shooterMotor.set(0);
            return;
        }

        double motorRPS = wheelRPS * ShooterConstants.GEAR_RATIO;
        shooterMotor.setControl(new VelocityVoltage(motorRPS));
    }

    /**
     * Sets the shooter motor speed.
     * 
     * @param speed
     */
    public void setShooterSpeed(double speed) {
        shooterMotor.set(speed);
    }
}