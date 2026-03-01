// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import swervelib.math.Matter;

/**
 * The Constants class provides a single place for robot-wide numerical and boolean constants.
 * This class should not contain logic. All values should be declared as public static fields.
 *
 * It is recommended to statically import this class (or one of its inner classes) where needed.
 */
public final class Constants {
  private Constants() {} // Prevent instantiation

  /* ================= Robot Physical Properties ================= */

  public static final class RobotProperties {
    private RobotProperties() {} // Prevent instantiation

    /** Robot mass in kilograms (measured weight minus bumpers). */
    public static final double ROBOT_MASS = Units.lbsToKilograms(148 - 20.3); // lbs to kg

    /** Center of mass used for swerve dynamics calculations. */
    public static final Matter CHASSIS = new Matter(
        new Translation3d(0, 0, Units.inchesToMeters(8)),
        ROBOT_MASS);

    /** Control loop period in seconds (20ms DS + ~110ms controller latency). */
    public static final double LOOP_TIME = 0.13;

    /** Maximum linear speed of the robot in meters per second. */
    public static final double MAX_SPEED = Units.feetToMeters(14.5);

    /** Distance from the camera to the front bumper in meters. */
    public static final double CAM_TO_BUMPER_DISTANCE = 0.33;
  }

  /* ================= Shooter ================= */

  public static final class ShooterConstants {
    private ShooterConstants() {} // Prevent instantiation

    /** Manual percent output command in the range [-1.0, 1.0]. */
    public static final double SHOOTER_POWER = 0.6;

    /** CAN ID for the primary shooter motor. */
    public static final int SHOOTER_MOTOR_ID1 = 0;

    /** CAN ID for the follower shooter motor. */
    public static final int SHOOTER_MOTOR_ID2 = 0;

    /** Gear ratio from motor to shooter wheel (motor rotations per wheel rotation). */
    public static final double GEAR_RATIO = 1.0; // Expected to be ~3:1 according to Rob

    /** Minimum valid distance to the target (in meters) used to clamp Limelight-derived values. */
    public static final double MINIMUM_DISTANCE = Units.inchesToMeters(31.5); // 90 inches to meters

    /** Maximum valid distance to the target (in meters) used to clamp Limelight-derived values. */
    public static final double MAXIMUM_DISTANCE = Units.inchesToMeters(250); // 250 inches to meters

    /** Gravity constant (m/s^2) used for projectile motion calculations. */
    public static final double GRAVITY_CONSTANT = 9.81;

    /** Physical shooter wheel radius in meters (used as the baseline before efficiency scaling). */
    public static final double SHOOTER_WHEEL_RADIUS = Units.inchesToMeters(2.25); // 2.25 inches to meters

    /** 
     * Key for tuning the effective radius efficiency on the dashboard. This allows live adjustments to
     * account for real-world losses without changing the physical constants. The value should be in the 
     * range (0, 1], where 1 means no losses and values closer to 0 represent more significant losses. 
     * Tuning this value effectively scales the computed wheel radius used in velocity calculations, 
     * allowing you to empirically match the theoretical projectile motion to actual shot performance.
     *
     * TUNING GUIDE:
     * 1) Set up the robot at a known, repeatable distance inside MINIMUM/MAXIMUM range.
     * 2) Command a shot and observe whether shots fall short or overshoot.
     * 3) Increase the value to raise the computed RPS (shots go farther).
     * 4) Decrease the value to lower the computed RPS (shots go shorter).
     * 5) Re-test at a few distances to confirm the curve stays consistent.
     */
    public static final double DEFAULT_RADIUS_EFFICIENCY = 0.85;

    /** Fixed shooter launch angle in radians (used in the projectile motion calculation). */
    public static final double SHOOTER_ANGLE = Math.toRadians(55.0); // Convert 55 degrees to radians

    /** Height of the shooter exit point above the floor, in meters. */
    public static final double SHOOTER_HEIGHT = Units.inchesToMeters(27); // 27 inches to meters

    /** Height of the target center above the floor, in meters. */
    public static final double GOAL_HEIGHT = Units.inchesToMeters(72); // 72 inches to meters

    /** Vertical offset between the target and the shooter exit point (meters). */
    public static final double HEIGHT_DIFFERENCE = GOAL_HEIGHT - SHOOTER_HEIGHT;
  }

  /* ================= Climb ================= */

  public static final class ClimbConstants {
    private ClimbConstants() {} // Prevent instantiation

    /** CAN ID for the climb motor. */
    public static final int CLIMB_MOTOR_ID = 0;

    /** Sensor-to-mechanism ratio (motor rotations per mechanism rotation). */
    public static final double CLIMB_SENSOR_TO_MECHANISM_RATIO = 1.0;

    /** Preset retracted position in mechanism rotations. */
    public static final double CLIMB_RETRACTED_ROTATIONS = 0.0;

    /** Preset extended position in mechanism rotations. */
    public static final double CLIMB_EXTENDED_ROTATIONS = 0.0;

    /**
     * Conversion factor from motor rotations to mechanism travel distance (e.g., inches).
     * Calculate as: (1 / Gear Ratio) * (Lead or Pitch Diameter * PI)
     * Example: 16:1 gearbox with 1-inch lead screw -> (1/16) * 1.0 = 0.0625 inches/rotation.
     * Defaulting to 1.0 as placeholder.
     */
    public static final double CLIMB_INCHES_PER_MOTOR_ROTATION = 1.0;

    /* ================= Motion Magic & PID ================= */

    /** Proportional Gain. Monitor error to tune this. */
    public static final double kP = 40.0;

    /** Integral Gain. Usually 0 for positioning. */
    public static final double kI = 0.0;

    /** Derivative Gain. Dampens oscillation. */
    public static final double kD = 0.0;

    /** Velocity Feedforward. */
    public static final double kV = 0.12;

    /** Static Friction Feedforward. */
    public static final double kS = 0.25;

    /** Gravity Feedforward. IMPORTANT: Tune this to hold the robot weight! */
    public static final double kG = 0.50; // Placeholder

    /** Cruise Velocity in rotations per second. */
    public static final double MM_CRUISE_VELOCITY = 80.0; // ~6000 RPM (motor max is ~6300 free speed)

    /** Acceleration in rotations per second squared. */
    public static final double MM_ACCELERATION = 160.0; // 0.5s to max speed

    /** Jerk in rotations per second cubed. Smoothing. */
    public static final double MM_JERK = 1600.0;

    /* ================= Safety ================= */

    /** Current limit in Amps to prevent motor burnout. */
    public static final double CURRENT_LIMIT = 60.0;

    /** Forward Soft Limit (Extension Limit). */
    public static final double SOFT_LIMIT_FORWARD = CLIMB_EXTENDED_ROTATIONS;

    /** Reverse Soft Limit (Retraction Limit). */
    public static final double SOFT_LIMIT_REVERSE = CLIMB_RETRACTED_ROTATIONS;
  }

  /* ================= Hopper ================= */

  public static final class HopperConstants {
    private HopperConstants() {} // Prevent instantiation

    /** CAN ID for the primary hopper motor. */
    public static final int HOPPER_MOTOR_ID1 = 0;

    /** CAN ID for the follower hopper motor. */
    public static final int HOPPER_MOTOR_ID2 = 0;
  }

  /* ================= Intake ================= */

  public static final class IntakeConstants {
    private IntakeConstants() {} // Prevent instantiation

    public static final int INTAKE_MOTOR_ID = 0;
  }

  /* ================= Object Recognition ================= */

  public static final class ObjectRecognitionConstants {
    private ObjectRecognitionConstants() {} // Prevent instantiation

    /** The name of the limelight used for object detection. */
    public static final String LIMELIGHT_NAME = "limelight";

    /** Pipeline index for standard AprilTag processing. */
    public static final int APRIL_TAG_PIPELINE_INDEX = 0;

    /** Pipeline index for neural network object detection. */
    public static final int OBJECT_DETECTION_PIPELINE_INDEX = 1;
  }

  /* ================= Drivebase ================= */

  public static final class DrivebaseConstants {
    private DrivebaseConstants() {} // Prevent instantiation

    /** Time to hold wheel lock after disable in seconds. */
    public static final double WHEEL_LOCK_TIME = 10.0;
  }

  /* ================= Operator Controls ================= */

  public static class OperatorConstants {
    private OperatorConstants() {} // Prevent instantiation

    /** Global joystick deadband to prevent stick drift. */
    public static final double DEADBAND = 0.1;

    /** Scalar applied to turning input for driver feel. */
    public static final double TURN_CONSTANT = 6.0;
  }

  /* ================= Elevator ================= */

  public static final class ElevatorConstants {
    private ElevatorConstants() {} // Prevent instantiation

    /** Spark MAX configuration for the elevator motor controller. */
    public static final SparkMaxConfig ELEVATOR_MOTOR_CONFIG = new SparkMaxConfig();

    static {
      // Basic motor behavior: brake when idle and enforce a current limit.
      ELEVATOR_MOTOR_CONFIG
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(60);

      // Closed-loop settings for position/velocity control (tune as needed).
      ELEVATOR_MOTOR_CONFIG.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.1, 0.0, 0.0)
          .outputRange(-1, 1);
    }
  }

  /* ================= Turret ================= */

  public static final class TurretConstants {
    private TurretConstants() {} // Prevent instantiation

    /** Maximum turret speed command in the range [-1.0, 1.0]. */
    public static final double MAX_TURRET_SPEED = 0.5; // Runtime code should clamp to [-0.5, 0.5].

    /** CAN ID for the turret motor. */
    public static final int TURRET_MOTOR_ID = 23;
  }
}
