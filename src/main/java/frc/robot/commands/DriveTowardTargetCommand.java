package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Constants.ObjectRecognitionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.util.Utils;

/**
 * A command that detects either an AprilTag or a game piece and then either aligns to the target
 * or drives the robot toward it.
 * 
 * This command uses the Limelight camera to detect an AprilTag or game piece and:
 *  - Rotates the robot to face the target.
 *  - Optionally drives forward until a certain distance is reached.
 */
public class DriveTowardTargetCommand extends Command {

    /**
     * Helper class to store detection state of the detected game piece.
     */
    private static class DetectionState {
        public String className = ""; // Name of the detected game piece
        public double confidence = 0.0; // Confidence of the detection
        public double distance = 0.0; // Calculated distance in meters
        public boolean hasTarget = false; // Whether a target was detected

        /**
         * Clears the detection state.
         */
        public void clear() {
            className = "";
            confidence = 0.0;
            distance = 0.0;
            hasTarget = false;
        }
    }

    // Current detection state of the detected game piece.
    private final DetectionState currentState = new DetectionState();

    // Limelight NetworkTable used to fetch target 3D pose data, used to calculate distance to target.
    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable(ObjectRecognitionConstants.LIMELIGHT_NAME);

    // Instance of the SwerveSubsystem to control the robot's movement
    private final SwerveSubsystem drivebase;

    // Maximum forward speed limit (m/s)
    private final double maxSpeed;

    // Maximum rotation speed limit (rad/s)
    private final double maxRotation;

    // Boolean to indicate if we are currently tracking an AprilTag or a game piece (object detection)
    private boolean detectAprilTag;

    // Desired stopping distance from the target (from bumper to tag) in meters. The robot will stop when it reaches this distance.
    private final double STOP_DISTANCE = 0.5;

    // Current forward and rotation speeds
    private double forwardSpeed = 0.0;
    private double rotationSpeed = 0.0;

    // Variables to store the latest tx, ty, tz values from the Limelight
    private double tx;

    /**
     * Constructor for DriveTowardTagCommand with specified max speeds (will must likely be used for
     * align-only with maxSpeed = 0). By default, it will use AprilTag detection. To use object
     * detection, use the other constructor.
     * 
     * @param drivebase   The swerve drive subsystem used to move the robot.
     * @param maxSpeed    The maximum forward speed in meters per second. Set to 0 for align-only.
     * @param maxRotation The maximum rotation speed in radians per second.
     */
    public DriveTowardTargetCommand(SwerveSubsystem drivebase, double maxSpeed, double maxRotation) {
        this.drivebase = drivebase;
        this.maxSpeed = maxSpeed;
        this.maxRotation = maxRotation;

        // Require the drivebase so no other drive commands run at the same time
        addRequirements(drivebase);
    }

    /**
     * Constructor for DriveTowardTagCommand that allows specifying whether to detect AprilTags or
     * use object detection. It uses default max speeds (2.0 m/s for forward, 2.0 rad/s for rotation).
     * 
     * @param drivebase      The swerve drive subsystem.
     * @param detectAprilTag If true, uses AprilTag detection; if false, uses object detection.
     */
    public DriveTowardTargetCommand(SwerveSubsystem drivebase, boolean detectAprilTag) {
        this.drivebase = drivebase;
        this.maxSpeed = 2.0;
        this.maxRotation = 2.0;
        this.detectAprilTag = detectAprilTag;

        // Require the drivebase so no other drive commands run at the same time
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        // Runs once when the command starts

        // Set the appropriate Limelight pipeline based on the detection mode
        if (detectAprilTag) {
            setPipelineToAprilTags();
        } else {
            setPipelineToObjectDetection();
        }

        // Reset detection state during initialization to avoid using stale data from previous runs
        currentState.clear();

        SmartDashboard.putString("LL Status", "Searching for target...");
    }

    @Override
    public void execute() {
        // Runs repeatedly while the command is active (approx. every 20ms)

        if (detectAprilTag) {
            // AprilTag Detection Logic

            // 1. Check if we have a valid fiducial ID to confirm we are seeing an AprilTag
            double fid = LimelightHelpers.getFiducialID(ObjectRecognitionConstants.LIMELIGHT_NAME);
            if (Double.isNaN(fid) || fid < 0.0) {
                SmartDashboard.putString("LL Status", "Tag ID not found");
                currentState.clear();
                stopRobot();
                return;
            }
        } else {
            // Object Detection Logic

            // Get the latest results from the Limelight camera with the detected game piece(s).
            LimelightResults results = LimelightHelpers.getLatestResults(ObjectRecognitionConstants.LIMELIGHT_NAME);

            // 1. Check for neural network detections
            if (results.targets_Detector == null || results.targets_Detector.length == 0) {
                SmartDashboard.putString("LL Status", "No object visible");
                currentState.clear();
                stopRobot();
                return;
            }

            // Pick the first detection (highest confidence usually sorted first by Limelight)
            LimelightTarget_Detector detection = results.targets_Detector[0];
            currentState.className = detection.className;
            currentState.confidence = detection.confidence;
        }

        // --- SHARED DETECTION LOGIC ---

        // 2. Get the "tv" (Target Valid) value from Limelight NetworkTables.
        // If tv < 1.0, it means no valid target is currently visible to the Limelight.
        double tv = limelight.getEntry("tv").getDouble(0.0);
        if (tv < 1.0) {
            SmartDashboard.putString("LL Status", "No target visible");
            currentState.clear();
            stopRobot();
            return;
        }

        // 3. Get the 3D pose of the target relative to the Limelight.
        // This is an array of 6 values: [x, y, z, roll, pitch, yaw].
        double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
        if (pose == null || pose.length < 3) {
            SmartDashboard.putString("LL Status", "Invalid pose data");
            currentState.clear();
            stopRobot();
            return;
        }

        // Extract tx coordinates from the pose. The other values (ty, tz) are available if needed, but are not used directly
        // for control in this command since we calculate distance using the Utils method.
        tx = pose[0];// Horizontal offset (left/right) in meters.

        // Calculate the straight-line distance using the Utils method, which accounts for the camera offset from the bumper.
        currentState.distance = Utils.calculateDistanceToTarget(limelight);
        currentState.hasTarget = true;

        // Update SmartDashboard for debugging
        SmartDashboard.putNumber("LL tx (m)", tx);
        SmartDashboard.putNumber("LL bumper->target (m)", currentState.distance);
        SmartDashboard.putString("LL Status", "Target Locked");

        // --- CONTROL LOGIC ---

        // A. Forward Speed Control
        // Only drive forward if we have a maxSpeed > 0 and are further away than our desired stop distance.
        if (maxSpeed > 0 && currentState.distance > STOP_DISTANCE) {
            // Proportional control: slow down as we get closer
            // We divide by 4.0 to create a gentle deceleration curve
            double speedFactor = Math.min(1.0, currentState.distance / 4.0);
            forwardSpeed = maxSpeed * speedFactor;
        } else {
            forwardSpeed = 0.0;
        }

        // B. Rotation Control
        // Turn to face the tag (minimize tx)
        double kP_turn = 1.0; // Proportional gain for turning
        rotationSpeed = -kP_turn * tx; // Negative because positive tx means tag is to the right, so we turn right
                                       // (negative Z rotation)

        // Clamp rotation speed to our maximum allowed limit
        rotationSpeed = Math.max(-maxRotation, Math.min(maxRotation, rotationSpeed));

        SmartDashboard.putNumber("LL ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("LL RotationSpeed", rotationSpeed);

        // Apply the calculated speeds to the robot
        // Translation2d(x, y) -> x is forward, y is left
        drivebase.drive(new Translation2d(forwardSpeed, 0), rotationSpeed, true);
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when the command finishes or is interrupted
        stopRobot();
        SmartDashboard.putString("LL Status", interrupted ? "Interrupted" : "Arrived at Target");
        System.out.println("[DriveTowardTarget] Ended");
    }

    @Override
    public boolean isFinished() {
        // Check if we should stop automatically based on target visibility and distance

        // 1. If we lost the target, stop the command (driver can re-press to try again)
        if (!currentState.hasTarget) {
            return true;
        }

        // 2. If we are driving forward (maxSpeed > 0) and reached the target, stop
        boolean reachedTarget = (maxSpeed > 0) && (currentState.distance <= STOP_DISTANCE);

        // Note: If maxSpeed is 0 (align only), we never "finish" based on distance.
        // The driver must release the button to stop aligning.

        return reachedTarget;
    }

    /**
     * Helper method to stop the robot completely.
     */
    private void stopRobot() {
        drivebase.drive(new Translation2d(0.0, 0.0), 0.0, true);
    }

    /**
     * Sets the Limelight pipeline to the AprilTag detection pipeline.
     */
    public void setPipelineToAprilTags() {
        LimelightHelpers.setPipelineIndex(ObjectRecognitionConstants.LIMELIGHT_NAME, ObjectRecognitionConstants.APRIL_TAG_PIPELINE_INDEX);
    }

    /**
     * Sets the Limelight pipeline to the object detection pipeline.
     */
    public void setPipelineToObjectDetection() {
        LimelightHelpers.setPipelineIndex(ObjectRecognitionConstants.LIMELIGHT_NAME, ObjectRecognitionConstants.OBJECT_DETECTION_PIPELINE_INDEX);
    }
}

// Per Madison's request, 300 lines =)