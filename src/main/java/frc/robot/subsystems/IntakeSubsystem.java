package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.kIntakeMotor,MotorType.kBrushless);
    
    public IntakeSubsystem() {}

    @Override
    public void periodic() {
    }

    public void setIntakeMotor(double velocity){
        intakeMotor.set(velocity);
    }

    public void stopIntakeMotor(){
        intakeMotor.set(0);
    }
}

