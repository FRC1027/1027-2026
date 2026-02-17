package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import frc.robot.util.Constants.RobotProperties;

/**
 * Shared utility methods used across robot subsystems and commands.
 */
public final class Utils {

  private Utils() {} // Prevent instantiation

  /**
   * Applies a deadband and rescales the remaining joystick range back to full scale.
   *
   * @param JoystickValue joystick input value (expected range: [-1.0, 1.0])
   * @param DeadbandCutOff deadband threshold (expected range: [0.0, 1.0))
   * @return 0.0 when inside the deadband, otherwise a rescaled value in [-1.0, 1.0]
   */
  public static double deadbandReturn(double JoystickValue, double DeadbandCutOff) {
    double deadbandReturn;

    if (JoystickValue < DeadbandCutOff && JoystickValue > (DeadbandCutOff * (-1))) {
      // Inside the deadband window: output zero.
      deadbandReturn = 0;
    } else {
      // Outside the deadband: remove offset while preserving sign, then rescale to full range.
      deadbandReturn = (JoystickValue - (Math.abs(JoystickValue) / JoystickValue * DeadbandCutOff)) / (1 - DeadbandCutOff);
    }
    return deadbandReturn;
  }

  /**
   * Calculates the Euclidean distance from the bumper to the target tag using Limelight data.
   * 
   * @return Distance from bumper to target tag in meters, or NaN if the Limelight pose is unavailable.
   */
  public static double calculateDistanceToTarget(NetworkTable limelight) {
    // Read the target pose in the camera coordinate frame (x = left/right, y = up/down, z = forward).
    double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);

    // Ensure we have valid pose data (at least 3 values for x, y, z).
    if (pose.length < 3) {
        return Double.NaN;
    }

    double tx = pose[0]; // Horizontal offset (left/right) in meters.
    double ty = pose[1]; // Vertical offset (up/down) in meters (unused for distance here).
    double tz = pose[2]; // Forward distance (depth) in meters.

    // Compute planar distance from camera to tag using X/Z components.
    double cameraToTag = Math.sqrt(tx * tx + tz * tz);

    // Convert camera-to-tag to bumper-to-tag by subtracting the camera offset.
    return Math.max(0.0, cameraToTag - RobotProperties.CAM_TO_BUMPER_DISTANCE);
  }
}
