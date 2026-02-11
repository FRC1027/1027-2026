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
public final class Constants
{
  private Constants() {} // Prevent instantiation

  /* ================= Robot Physical Properties ================= */

  /** Robot geometry and physical characterization constants. */
  public static final class RobotProperties {
    
    private RobotProperties() {} // Prevent instantiation

    /** Robot mass in kilograms (measured weight minus bumpers). */
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // lbs to kg

    /** Center of mass used for swerve dynamics calculations. */
    public static final Matter CHASSIS =
        new Matter(
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

  /** Shooter subsystem constants. */
  public static final class ShooterConstants {

    private ShooterConstants() {} // Prevent instantiation

    /** Manual percent output command in the range [-1.0, 1.0]. */
    public static final double SHOOTER_POWER = 0.6;

    /** Duration to run shooter for intake/outtake in seconds. */
    public static final double SHOOTER_TIME = 10.0;

    /** CAN ID for the primary shooter motor. */
    public static final int SHOOTER_MOTOR_ID1 = 23;

    /** CAN ID for the follower shooter motor (set to a real ID when used). */
    public static final int SHOOTER_MOTOR_ID2 = 0;

    /** Gear ratio from motor to shooter wheel (motor rotations per wheel rotation). */
    public static final double GEAR_RATIO = 1.0; // Expected to be ~3:1 according to Rob

    /** Default target wheel RPS for fixed-speed shooting. */
    public static final double SHOOTER_TARGET_RPS = 24.667;
  }


  /* ================= Turret ================= */

  /** Turret subsystem constants. */
  public static final class TurretConstants {

    private TurretConstants() {} // Prevent instantiation

    /** Maximum turret speed command in the range [-1.0, 1.0]. */
    public static final double MAX_TURRET_SPEED = 0.5; // Runtime code should clamp to [-0.5, 0.5].

    /** CAN ID for the turret motor. */
    public static final int TURRET_MOTOR_ID = 23;
  }


  /* ================= Drivebase ================= */

  /** Drivebase-specific constants. */
  public static final class DrivebaseConstants {

    private DrivebaseConstants() {} // Prevent instantiation

    /** Time to hold wheel lock after disable in seconds. */
    public static final double WHEEL_LOCK_TIME = 10.0;
  }


  /* ================= Operator Controls ================= */

  /** Driver and operator input constants. */
  public static class OperatorConstants {

    private OperatorConstants() {} // Prevent instantiation

    /** Global joystick deadband to prevent stick drift. */
    public static final double DEADBAND = 0.1;

    /** Deadband for the left Y axis (forward/back). */
    public static final double LEFT_Y_DEADBAND = 0.1;
    
    /** Deadband for the right X axis (rotation). */
    public static final double RIGHT_X_DEADBAND = 0.1;

    /** Scalar applied to turning input for driver feel. */
    public static final double TURN_CONSTANT = 6.0;
  }


  /* ================= Elevator ================= */

  /** Elevator subsystem constants and motor controller configuration. */
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
}
