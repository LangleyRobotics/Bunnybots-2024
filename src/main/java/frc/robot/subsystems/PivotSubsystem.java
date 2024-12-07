package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.MathMethods;

public class PivotSubsystem extends SubsystemBase {

    private final CANBus canbus = new CANBus("yes");
    private final TalonFX pivotMotor = new TalonFX(PivotConstants.kPivotMotorPort, canbus);
    private final DutyCycleEncoder pivotAbsEncoder = new DutyCycleEncoder(0,360,0);

    private final ProfiledPIDController pivotPIDController = new ProfiledPIDController(PivotConstants.kP_Pivot, PivotConstants.kI_Pivot, PivotConstants.kD_Pivot, new TrapezoidProfile.Constraints(100, 50));
    double goal = 0;
    double offset = 0;


    public PivotSubsystem() {
        pivotPIDController.setTolerance(ElevatorConstants.deadbandAngle);
        pivotMotor.setPosition(0);

        // Configures the kraken like how REV Hardware Client would for Sparkmaxes
        TalonFXConfigurator pivotConfig = pivotMotor.getConfigurator();
        pivotConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        pivotConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(80)
                        .withStatorCurrentLimitEnable(true));
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Talon Position", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Talon Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Goal Position", goal);  
    }

    //Make pivot motor run
    public void setPivotMotorNoBounds(double velocity) {
        pivotMotor.set(velocity);
    }

    public void setPivotMotor(double velocity){

        try {
            if (getPivotAbsEncoder() < PivotConstants.kMaxPivotPosition || 
                getPivotAbsEncoder() > PivotConstants.kMinPivotPosition) {
                    //motors are facing opposite directions
                    pivotMotor.set(velocity);
                } else {
                    pivotMotor.set(0.0);
                }
        } catch(Exception e) {
             System.out.println("Error: Pivot Motor is Set to a value out of valid range [-1.0, 1.0]");
        }
    }
    

    public double getPivotAbsEncoder() {
        return pivotAbsEncoder.get() - offset;
    }
    public double getPivotPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }
    public void stopPivotMotor() {
        pivotMotor.set(0);
      }

    //reset the pivot encoder to the be 0 at down position
    public void resetPivotEncoder() {
        offset = pivotAbsEncoder.get();
    }

    public void pidPivot() {
        double motorVoltage = pivotPIDController.calculate(getPivotPosition(), goal);// + elevatorFeedForward.calculate();
        //try elevatorMotor.setPosition(double position);
        pivotMotor.setVoltage(motorVoltage);
    }

    public void setGoTo(double setpoint) {
        this.goal = setpoint;
        pivotMotor.setPosition(setpoint);
    }

    public void setGoal(double setpoint) {
        this.goal = setpoint;
    }

    public double getGoal() {
        return goal;
    }


    //Send arm to a specific setpoint
    public void goToSetpoint(double desPosition) {
        //desPosition = radians
        double output = pivotPIDController.calculate(getPivotAbsEncoder(), desPosition);
        if(output > 0 && getPivotAbsEncoder() >= 155) { 
            // pivot going forward, slow so doesn't hit top of arm
            pivotMotor.setVoltage(0.4);
        } else if(output < 0 && getPivotAbsEncoder() <= 45) { 
            // pivot going backwards, slow so doesnt hit bottom of arm
            pivotMotor.setVoltage(-0.4);
        } else if(Math.abs(getPivotAbsEncoder() - desPosition) > PivotConstants.deadbandAngle){
            // pivot is freeeee
            pivotMotor.setVoltage(output);
        }
    }

    public boolean isAtSetpoint() {
        return pivotPIDController.atSetpoint();
    }

}