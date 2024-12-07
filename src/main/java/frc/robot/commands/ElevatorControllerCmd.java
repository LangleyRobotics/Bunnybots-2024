// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.MathMethods;


public class ElevatorControllerCmd extends Command {
  
  private final ElevatorSubsystem elevatorSubsystem;
  private final Supplier<Boolean> elevatorPositiveDirFunction;
  private final Supplier<Boolean> elevatorNegativeDirFunction;

  
  public ElevatorControllerCmd(ElevatorSubsystem elevatorSubsystem, Supplier<Boolean> elevatorPositiveDirFunction, Supplier<Boolean> elevatorNegativeDirFunction) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorPositiveDirFunction = elevatorPositiveDirFunction;
    this.elevatorNegativeDirFunction = elevatorNegativeDirFunction;

    addRequirements(elevatorSubsystem);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    
    //are the buttons pressed
    boolean positiveDir = elevatorPositiveDirFunction.get();
    boolean negativeDir = elevatorNegativeDirFunction.get();

    //double velocity = 0;

    elevatorSubsystem.pidElev();

    if(positiveDir){
      elevatorSubsystem.setGoal(elevatorSubsystem.getGoal()+0.3);
    }
    else if(negativeDir){
      elevatorSubsystem.setGoal(elevatorSubsystem.getGoal()-0.3);
    }


    
    if(elevatorSubsystem.getGoal() < 0){
      elevatorSubsystem.setGoal(0.1);
    }
    if(elevatorSubsystem.getGoal() > 60.8){ 
      elevatorSubsystem.setGoal(60.7);

    }

    // //Up elevator
    // if(positiveDir) {
    //   // elevatorSubsystem.setElevatorMotor(ElevatorConstants.kElevatorMotorSpeed);
    //   elevatorSubsystem.pidElev();
    // } 
    
    // //Down elevator
    // else if(negativeDir) {
    //   // elevatorSubsystem.setElevatorMotor(-ElevatorConstants.kElevatorMotorSpeed);
    //   elevatorSubsystem.pidElev();
    // } 
    
    //Keep the elevator at the same height
    // else {
    //   elevatorSubsystem.setElevatorMotorNoBounds(
    //     MathMethods.signDouble(Math.cos(elevatorSubsystem.getElevatorPosition())) * 0.02 
    //     - ElevatorConstants.elevatorCompensation * Math.cos(Math.toRadians(elevatorSubsystem.getElevatorPosition())));
    // }

    
  }


  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopElevatorMotor();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
