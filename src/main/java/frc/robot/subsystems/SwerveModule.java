// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.MathMethods;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.*;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;



public class SwerveModule extends SubsystemBase{

  private boolean isConfig = false;

  private final SparkMax driveMotor;
  private final SparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;
  private final SparkMaxConfig driveSparkMaxConfig;
  private final SparkMaxConfig turningSparkMaxConfig;
  private final EncoderConfig driveEncoderConfig;
  private final EncoderConfig turningEncoderConfig;


  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  private final CANcoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;
  

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  private final PIDController turningPidController;
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveMotorReversed Whether the drive encoder is reversed.
   * @param turningMotorReversed Whether the turning encoder is reversed.
   * @param absoluteEncoderChannel The channel of the absolute encoder
   * @param absoluteEncoderReversed Whether the absolute encoder is reversed
   * @param absoluteEncoderOffsetRad The offset (radians) of absolute encoder
   * @param relEncoderInverted
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      int absoluteEncoderChannel,
      boolean absoluteEncoderReversed,
      double absoluteEncoderOffsetRad) {


    this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    
    driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    driveSparkMaxConfig = new SparkMaxConfig();
    turningSparkMaxConfig = new SparkMaxConfig();
    driveEncoderConfig = new EncoderConfig();
    turningEncoderConfig = new EncoderConfig();

    driveEncoderConfig.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoderConfig.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoderConfig.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoderConfig.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    driveSparkMaxConfig.apply(driveEncoderConfig);
    turningSparkMaxConfig.apply(turningEncoderConfig);

    if(driveMotor.configure(driveSparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters) == REVLibError.kOk) {
      isConfig = true;
    }
    turningMotor.configure(turningSparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();


    absoluteEncoder = new CANcoder(absoluteEncoderChannel);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

  

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }


  @Override
    public void periodic() {
        SmartDashboard.putBoolean("Motors Configured?", isConfig);   
    }


  public double getDrivePosition() {
      return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
      return MathMethods.moduloAngle((turningEncoder.getPosition()));
  }

  public double getDriveVelocity() {
      return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
      return turningEncoder.getVelocity();
  }

  public boolean getTurningMotorInvert() {
    return turningMotor.getInverted();
  }

  

  public double getAbsoluteEncoderRad() {
    StatusSignal<Angle> absAngle = absoluteEncoder.getAbsolutePosition();
    absAngle = absAngle.refresh();
    double angle = absAngle.getValueAsDouble() * 90/15*(90/86.1);
    angle -= absoluteEncoderOffsetRad;
    return MathMethods.moduloAngle(angle * (absoluteEncoderReversed ? -1.0 : 1.0));
  }

  /** Resets all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
          getDrivePosition(), new Rotation2d(getTurningPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;
    }
    state.optimize(getState().angle);
    driveMotor.set(state.speedMetersPerSecond / Constants.kMaxSpeedMetersPerSecond);
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "]  state", state.toString());
    SmartDashboard.putNumber(absoluteEncoder.getDeviceID() + "desired speed", state.speedMetersPerSecond);
    SmartDashboard.putNumber(absoluteEncoder.getDeviceID() + "desired rotation", state.angle.getDegrees());
  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
    
  }
}