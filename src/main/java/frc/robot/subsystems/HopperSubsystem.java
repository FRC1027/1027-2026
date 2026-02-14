package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

private final TalonFX hopperMotor1;
private final TalonFX hopperMotor2;

    public HopperSubsystem(){
        hopperMotor1 = new TalonFX(HopperConstants.HOPPER_MOTOR_ID1);
        hopperMotor2 = new TalonFX(HopperConstants.HOPPER_MOTOR_ID2);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.15; // Proportional gain for velocity control (tuning may be required).
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        hopperMotor1.getConfigurator().apply(config);
        hopperMotor2.getConfigurator().apply(config);

        hopperMotor2.setControl(new Follower(HopperConstants.HOPPER_MOTOR_ID1, MotorAlignmentValue.Opposed)); // Check if it should be Opposed or Aligned
    }
}
