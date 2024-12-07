// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.MathMethods;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ElevatorSubsystem extends SubsystemBase{
    
    private final CANBus canbus = new CANBus("yes");
    private final TalonFX elevatorMotorRight = new TalonFX(ElevatorConstants.kElevatorMotorRightPort, canbus);
    private final TalonFX elevatorMotorLeft = new TalonFX(ElevatorConstants.kElevatorMotorLeftPort, canbus);
    private final DutyCycleEncoder elevatorAbsEncoder = new DutyCycleEncoder(1,2000,0);

    private final ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(ElevatorConstants.kS_Elevator, ElevatorConstants.kG_Elevator, ElevatorConstants.kV_Elevator, ElevatorConstants.kA_Elevator);
    private final PIDController elevatorPIDController = new PIDController(ElevatorConstants.kP_Elevator, ElevatorConstants.kI_Elevator, ElevatorConstants.kD_Elevator);
    private final ProfiledPIDController profiledPIDController = new ProfiledPIDController(
        ElevatorConstants.kP_Elevator,
        ElevatorConstants.kI_Elevator,
        ElevatorConstants.kD_Elevator,
        // The motion profile constraints
        new TrapezoidProfile.Constraints(200, 100));

    double goal = 0;
    double absEncoderRaw = 0;
    double offset = 0;
    

    public ElevatorSubsystem() {
        // Configures the encoder to return a distance for every rotation
        //offset = elevatorAbsEncoder.get() - ElevatorConstants.kElevatorOffset;
        elevatorPIDController.setTolerance(ElevatorConstants.deadbandAngle);
        elevatorMotorLeft.setPosition(0);


        TalonFXConfigurator elevatorConfigRight = elevatorMotorRight.getConfigurator();
        elevatorConfigRight.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        elevatorConfigRight.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(80)
                        .withStatorCurrentLimitEnable(true));

        TalonFXConfigurator elevatorConfigLeft = elevatorMotorLeft.getConfigurator();
        elevatorConfigLeft.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        elevatorConfigLeft.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(80)
                        .withStatorCurrentLimitEnable(true));
        elevatorMotorRight.setControl(new Follower(ElevatorConstants.kElevatorMotorLeftPort,true));
    }


    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", getElevatorAbsEncoder());
        SmartDashboard.putBoolean("Elevator Encoder Connected?", elevatorAbsEncoder.isConnected());
        SmartDashboard.putNumber("Elevator Talon Position", elevatorMotorLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Talon Voltage", elevatorMotorLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Goal Position", goal);
    }

    public void setElevatorMotorNoBounds(double velocity) {
        //elevatorMotorRight.set(velocity);
        elevatorMotorLeft.set(-velocity);
    }

    //Manually controlling angle of elevator (operator controller during teleop)
    public void setElevatorMotor(double velocity){

        try {
            if (getElevatorPosition() < ElevatorConstants.kMaxElevatorPosition || 
                getElevatorPosition() > ElevatorConstants.kMinElevatorPosition) {
                    //motors are facing opposite directions
                   // elevatorMotorRight.set(velocity);
                    elevatorMotorLeft.set(-velocity);
                } else {
                    //elevatorMotorRight.set(0);
                    elevatorMotorLeft.set(0);
                }
        } catch(Exception e) {
            // System.out.println("Error: Elevator Motor is Set to a value out of valid range [-1.0, 1.0]");
        }
    }


    public void stopElevatorMotor(){
        //elevatorMotorRight.set(0);
        elevatorMotorLeft.set(0);
    }

    public double getElevatorPosition() {
        return elevatorMotorLeft.getPosition().getValueAsDouble();
    }

    //returns the value of the elevator encoder (position of the arm angle)
    public double getElevatorAbsEncoder() {
        return elevatorAbsEncoder.get() - offset;
    }

  

    public void builtInPidElev(double goal){}
        // create a position closed-loop request, voltage output, slot 0 configs
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

        // set position to 10 rotations
        //elevatorMotorLeft.setControl(m_request.withPosition(10));

    public void pidElevBAD() {
        // Use the output (and optionally the setpoint) here
        // double feedforward = 0;
        // elevatorMotorLeft.setVoltage(output + feedforward);
        // elevatorMotorRight.setVoltage(-(output + feedforward));
       
        double volt = profiledPIDController.calculate(getElevatorPosition()) + elevatorFeedForward.calculate(MetersPerSecond.of(profiledPIDController.getSetpoint().velocity)).in(Volts);
        elevatorMotorLeft.setVoltage(volt);
        //elevatorMotorRight.setVoltage(-volt);
    }

    public void setGoTo(double setpoint) {
        this.goal = setpoint;
        elevatorMotorLeft.setPosition(setpoint);
    }

    public void pidElev() {
        // Use the output (and optionally the setpoint) here
        // double feedforward = 0;
        // elevatorMotorLeft.setVoltage(output + feedforward);
        // elevatorMotorRight.setVoltage(-(output + feedforward));
        double motorVoltage = profiledPIDController.calculate(getElevatorPosition(), goal);// + elevatorFeedForward.calculate();
        //try elevatorMotor.setPosition(double position);
        elevatorMotorLeft.setVoltage(motorVoltage);
        //elevatorMotorRight.setVoltage(-motorVoltage);
    }
    public void setGoal(double setpoint) {
        this.goal = setpoint;
    }
    public double getGoal() {
        return goal;
    }

    public void setPosition(double setpoint) {
        profiledPIDController.setGoal(setpoint);
    }

    public Command updatePosition(Supplier<Double> setpoint) {
            return new RunCommand(() -> setPosition(setpoint.get()));
    }
    public boolean isAtSetpoint() {
        return profiledPIDController.atGoal();
    }

    // //Send elevator to a specific setpoint
    // public void goToSetpoint(double desPosition) {
    //     //desPosition = radians
    //     double output = elevatorPIDController.calculate(getElevatorAbsEncoder(), desPosition);
    //     if(output > 0 && getElevatorAbsEncoder() >= 155) { 
    //         // elevator going up, slow so doesnt hit top
    //         elevatorMotorRight.setVoltage(-0.4);
    //         elevatorMotorLeft.setVoltage(0.4);
    //     } else if(desPosition < ElevatorConstants.kAmpPosition && output < 0) { 
    //         // elevator going down, slow so doesnt hit bottom
    //         setPivotMotorNoBounds(MathMethods.signDouble(Math.cos(getElevatorAbsEncoder()))*0.02 - ElevatorConstants.pivotCompensation * Math.cos(Math.toRadians(getElevatorAbsEncoder())));
    //     } else if(Math.abs(getElevatorAbsEncoder() - desPosition) > ElevatorConstants.deadbandAngle){
    //         // pivot is freeeee
    //         elevatorMotorRight.setVoltage(-output);
    //         elevatorMotorLeft.setVoltage(output);
    //     }
    //     // System.out.println("output: " + output);
    //     // System.out.println("actual, target " + getPivotAbsEncoder() + ", " + desPosition);
    // }

    // public boolean isAtSetpoint() {
    //     return elevatorPIDController.atSetpoint();
    // }

    //TEST Feedforward and PID controller for pivot
    public void elevatorWithPID(double desPosition) {
        //desPosition = radians
        double output = elevatorPIDController.calculate(getElevatorAbsEncoder(), desPosition);
        if(output > 0 && desPosition <= 60) {
            //elevatorMotorRight.setVoltage(-0.03);
            elevatorMotorLeft.setVoltage(0.03);
        } else if(output < 0 && desPosition > 200) {
            setElevatorMotorNoBounds(MathMethods.signDouble(Math.cos(getElevatorAbsEncoder()))*0.02 - ElevatorConstants.elevatorCompensation * Math.cos(Math.toRadians(getElevatorAbsEncoder())));
        } else if (output > 0) {
            //elevatorMotorRight.setVoltage(-1*output);
            elevatorMotorLeft.setVoltage(1*output);
        } else if(output < 0) {
            //elevatorMotorRight.setVoltage(-4*output);
            elevatorMotorLeft.setVoltage(4*output);
        }
      }

}
