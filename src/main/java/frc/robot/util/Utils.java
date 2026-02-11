package frc.robot.util;

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
      deadbandReturn = (JoystickValue -
      (Math.abs(JoystickValue) / JoystickValue
      * DeadbandCutOff
      )
      )
      / (1 - DeadbandCutOff);
    }
    return deadbandReturn;
  }
}
