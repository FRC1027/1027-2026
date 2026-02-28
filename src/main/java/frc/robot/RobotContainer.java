// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.auto.AutoShootAtTag4;
import frc.robot.commands.DriveTowardTargetCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Constants.OperatorConstants;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Driver and Mechanism controllers
  public static final CommandXboxController driverXbox = new CommandXboxController(0);
  public static final CommandXboxController mechXbox = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve")
  );

  // Defining the HopperSubsystem
  private final HopperSubsystem m_hopper = new HopperSubsystem();

  // Defining the IntakeSubsystem
  private final IntakeSubsystem m_intake = new IntakeSubsystem(m_hopper::getHopperEnlarged);

  // Defining the ShooterSubsystem
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // Defining DriveTowardTagCommand (Drive Mode) ** Uses AprilTag detection **
  private final DriveTowardTargetCommand m_DriveTowardTagCommand = new DriveTowardTargetCommand(drivebase, true);

  // Defining DriveTowardGamePieceCommand (Drive Mode) ** Uses object detection **
  private final DriveTowardTargetCommand m_DriveTowardGamePieceCommand = new DriveTowardTargetCommand(drivebase, false);

  // Defining AlignTagCommand (Align Only Mode - MaxSpeed = 0) ** Only uses AprilTag detection, not object detection **
  private final DriveTowardTargetCommand m_AlignTagCommand = new DriveTowardTargetCommand(drivebase, 0.0, 2.0);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
                                                            //.withControllerRotationAxis(driverXbox::getRightX)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy().withControllerHeadingAxis(
      () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
      () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(0));

  // Constructs a SendableChooser variable that allows auto commands to be sent to the dashboard via NetworkTables
  private final SendableChooser<Command> m_chooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure/register any and all future commands here
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Creates a SendableChooser that adds all of the PathPlanner Autos to the dashboard
    m_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_chooser);

    /**
     * Add pathplanner commands to the dashboard with the following line of code:
     * 
     * m_chooser.addOption("Name of Command", m_insertNameHere);
     */

    // Add other auotonomous paths here so they can appear on the dashboard
    m_chooser.addOption("Shoot At Tag 4", new AutoShootAtTag4(drivebase, m_shooter));

    // Default Auto: Drive forward ~1 foot, then stop
    m_chooser.setDefaultOption("Drive Forward 1ft (Default)",
        Commands.run(() -> drivebase.drive(
        new Translation2d(0.25, 0.0),               // forward 0.25 m/s
              0.0,                             // no rotation
              true), drivebase)           // field-relative
      .withTimeout(Units.feetToMeters(1) / 0.25)   // ~1 foot distance
            .andThen(() -> drivebase.drive(
                new Translation2d(0.0, 0.0),
                0.0,
              true))                      // stop robot
    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

    /* ================= Mechanism Control Bindings ================= */

    // Controls the hooper with the mechXbox bumpers
    mechXbox.rightBumper().whileTrue(m_hopper.manualHopperControl());
    mechXbox.leftBumper().whileTrue(m_hopper.manualHopperControl());

    // Controls the enlargment/retraction of the hopper with the `a` button on the mechXbox
    mechXbox.a().onTrue(m_hopper.hopperEnlarger2000Command());

    // Controls the intake with the mechXbox triggers with a custom threshold for better responsiveness.
    mechXbox.rightTrigger(0.1).whileTrue(m_intake.manualIntakeCommand());
    mechXbox.leftTrigger(0.1).whileTrue(m_intake.manualIntakeCommand());

    // Controls the intake to run continuously via the `x` button (toggle-controlled), only while the hopper is enlarged.
    mechXbox.x().onTrue(m_intake.continuousIntakeCommand());

    // Controls the shooter with the `y` button on the mechXbox
    mechXbox.y().whileTrue(m_shooter.shoot());

    // Controls the shooter to align and shoot at a target tag with the `b` button on the mechXbox
    mechXbox.b().whileTrue(m_shooter.shootAlign(drivebase));

    /* ================= Driver Control Bindings ================= */

    // Controls the robot to drive and align toward a target tag with the `b` button on the driverXbox
    driverXbox.b().whileTrue(m_DriveTowardTagCommand);

    // Controls the robot to drive and align toward a game piece with the `y` button on the driverXbox
    driverXbox.y().whileTrue(m_DriveTowardGamePieceCommand);

    // Controls the robot to align with a target tag with the `x` button on the driverXbox
    driverXbox.x().whileTrue(m_AlignTagCommand);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
          Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(5, 2)),
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
      
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
    }

    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.y().onTrue(Commands.runOnce(drivebase::centerModulesCommand));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();

    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
// test