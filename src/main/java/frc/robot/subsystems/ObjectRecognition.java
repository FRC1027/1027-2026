package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ObjectRecognitionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.util.Utils;

/**
 * The ObjectRecognition subsystem interface for the Limelight camera to
 * identify game pieces. It uses a neural network pipeline to detect "Fuel"
 * game pieces, allowing the robot to automate game piece acquisition.
 */
public class ObjectRecognition extends SubsystemBase {

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

    // Limelight NetworkTable used to fetch target 3D pose data, used to calculate distance to target.
    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable(ObjectRecognitionConstants.LIMELIGHT_NAME);

    // Current detection state of the detected game piece.
    private final DetectionState currentState = new DetectionState();

    /**
     * Creates a new ObjectRecognition subsystem.
     */
    public ObjectRecognition() {
        // Initialize or configure Limelight here if needed
        // For example, ensuring LEDs are off by default
        LimelightHelpers.setLEDMode_ForceOff(ObjectRecognitionConstants.LIMELIGHT_NAME);
    }

    /**
     * Reads the latest results from the Limelight camera and updates the internal
     * state.
     * 
     * This method processes the neural network results to find the best target
     * (currently just the first one, but could be filtered by confidence/size).
     */
    public void updateInputs() {
        // Get the latest results from the Limelight camera regarding the detected game piece(s).
        LimelightResults results = LimelightHelpers.getLatestResults(ObjectRecognitionConstants.LIMELIGHT_NAME);

        // Check for neural network detections
        if (results.targets_Detector != null && results.targets_Detector.length > 0) {
            // Pick the first detection (highest confidence usually sorted first by Limelight)
            LimelightTarget_Detector detection = results.targets_Detector[0];

            currentState.hasTarget = true;
            currentState.className = detection.className;
            currentState.confidence = detection.confidence;

            // Calculate distance using 3D pose (target relative to camera)
            currentState.distance = Utils.calculateDistanceToTarget(limelight);

            // Logging for debugging (can be removed for production)
            System.out.println("Detected: " + currentState.className + " Distance: " + currentState.distance);
        } else {
            currentState.clear();
        }
    }

    /**
     * Command to switch pipeline, wait for it, read detections, and switch back.
     * 
     * @return A sequential command group.
     */
    public Command recognizeObjectsCommand() {
        return new InstantCommand(() -> setPipelineToObjectDetection()) // Switch to object detection pipeline
                .andThen(Commands.waitUntil(() -> LimelightHelpers.getCurrentPipelineIndex(
                        ObjectRecognitionConstants.LIMELIGHT_NAME) == ObjectRecognitionConstants.OBJECT_DETECTION_PIPELINE_INDEX))
                .andThen(new InstantCommand(this::updateInputs)) // Update state
                .andThen(new InstantCommand(() -> {
                    if (currentState.hasTarget) {
                        System.out.println(currentState.className + " detected at "
                                + String.format("%.2f", currentState.distance) + "m");
                    } else {
                        System.out.println("No object detected");
                    }
                }))
                .andThen(new InstantCommand(() -> setPipelineToAprilTags())) // Switch back to AprilTag pipeline
                .andThen(Commands.waitUntil(() -> LimelightHelpers.getCurrentPipelineIndex(
                        ObjectRecognitionConstants.LIMELIGHT_NAME) == ObjectRecognitionConstants.APRIL_TAG_PIPELINE_INDEX));
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

    @Override
    public void periodic() {
        // Optionally update inputs periodically if we want continuous tracking
        updateInputs();
    }
}