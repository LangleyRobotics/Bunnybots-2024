// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

//Set pivot to specific setpoints (intake, shoot up close, amp scoring)

public class SetPivotCmd extends Command {
  
  private final PivotSubsystem pivotSubsystem;
  private final int position;
  private final double targetPosition;

  public SetPivotCmd(PivotSubsystem pivotSubsystem, int position) {
    this.pivotSubsystem = pivotSubsystem;
    this.position = position;

    if(position == 0) {
      //**Up Right Intake position**
      this.targetPosition = PivotConstants.upRightIntake;
    } else if (position == 1) {
      //**Knocked Over Intake position**
      this.targetPosition = PivotConstants.knockedOverIntake;
    } else if (position == 2) {
      //**Starting position**
      this.targetPosition = PivotConstants.kMaxPivotPosition;
    } else if (position == 3) {
      //*Vault position*
      this.targetPosition = PivotConstants.vaultIntake;
    } else if (position == 4) {
      //*Stack outtake position*
      this.targetPosition = PivotConstants.stackOuttake;
    } else if (position == 5) {
        //*Stack outtake position*
        this.targetPosition = PivotConstants.HighOuttake;
    } else if (position == 6) {
      //*Auto outtake position*
      this.targetPosition = PivotConstants.autoOuttake;
    } else {
      this.targetPosition = PivotConstants.kMaxPivotPosition;
    }

    addRequirements(pivotSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    pivotSubsystem.setGoal(targetPosition);
  }

  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.stopPivotMotor();
  }

  @Override

  public boolean isFinished() {
    return pivotSubsystem.isAtSetpoint();
  }
}
