// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import javax.sound.midi.Sequencer;

import org.ejml.dense.row.decompose.TriangularSolver_CDRM;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AllForNaught;
//import frc.robot.Trajectories;
import frc.robot.commands.ElevatorControllerCmd;
//Auto Commands
import frc.robot.commands.IntakeAutoCmd;
import frc.robot.commands.IntakeControllerCmd;
import frc.robot.commands.PivotControllerCmd;
import frc.robot.commands.RumbleCmd;
import frc.robot.commands.SetElevatorCmd;
import frc.robot.commands.SetPivotCmd;
import frc.robot.commands.SwerveControllerCmd;
//Subsystem Imports
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

 



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();


  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.kSecondaryControllerPort);
  
 SendableChooser<Command> autoChooser = new SendableChooser<>();






  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  
    // Configure default commands

     
    robotDrive.setDefaultCommand(
        new SwerveControllerCmd(
            robotDrive,
            () -> -driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> driverController.getRightX(),
            () -> true, // field oriented
            () -> false));

    intakeSubsystem.setDefaultCommand(
      new IntakeControllerCmd(
        intakeSubsystem, 
        () -> 0.0, 
        0));
        

    pivotSubsystem.setDefaultCommand(
      new PivotControllerCmd(
        pivotSubsystem, 
        () -> operatorController.getLeftTriggerAxis(),
        () -> operatorController.getRightTriggerAxis()));

    elevatorSubsystem.setDefaultCommand(
      new ElevatorControllerCmd(
        elevatorSubsystem, 
        () -> false,
        () -> false));

    // robotDrive.zeroHeading();

    //lightingSubsystem.setDefaultCommand(lightingSubsystem.splitColor(Color.kAquamarine, Color.kDarkCyan));



    // Register Named Commands
    // Named commands = commands other than driving around that still need to be executed in auto

    var pivotToUpRightIntake = new SetPivotCmd(pivotSubsystem, 0).withTimeout(2);
    var pivotToKnockedOverIntake = new SetPivotCmd(pivotSubsystem, 1).withTimeout(2);
    var pivotToStartingPosition = new SetPivotCmd(pivotSubsystem, 3).withTimeout(2);
    var pivotToStackOuttakePosition = new SetPivotCmd(pivotSubsystem, 4).withTimeout(2);

    var elevatatorToBottom = new SetElevatorCmd(elevatorSubsystem, 1).withTimeout(1.5);
    var elevatatorToMiddle = new SetElevatorCmd(elevatorSubsystem, 0).withTimeout(1.5);
    var elevatatorToTop = new SetElevatorCmd(elevatorSubsystem, 2).withTimeout(1.5);

    var outtake = new IntakeAutoCmd(intakeSubsystem, -1).withTimeout(2.5);
    var intake = new IntakeAutoCmd(intakeSubsystem, 1).withTimeout(2.5);
    


    //Named Commands for PathPlanner
    NamedCommands.registerCommand("Pivot To Up Right Intake", pivotToUpRightIntake);
    NamedCommands.registerCommand("Pivot To Knocked Over Intake", pivotToKnockedOverIntake);
    NamedCommands.registerCommand("Pivot To Starting Position", pivotToStartingPosition);
    NamedCommands.registerCommand("Pivot To Stack Outtake Position", pivotToStackOuttakePosition);

    NamedCommands.registerCommand("Elevator To Bottom", elevatatorToBottom);
    NamedCommands.registerCommand("Elevator To Middle", elevatatorToMiddle);
    NamedCommands.registerCommand("Elevator To Top", elevatatorToTop);

    NamedCommands.registerCommand("Intake", intake);
    NamedCommands.registerCommand("Outtake", outtake);


    // Configure the button bindings
    configureButtonBindings();

    var pivotAutoOuttake = new SequentialCommandGroup(
      new SetPivotCmd(pivotSubsystem, 6).withTimeout(0.2),
      new PivotControllerCmd(pivotSubsystem).withTimeout(1)).withTimeout(1.2);
    var outtake2 = new IntakeAutoCmd(intakeSubsystem, 1).withTimeout(1.5);

   

    SequentialCommandGroup goStraight = robotDrive.AutoCommandFactory(Trajectories.goStraight);
    SequentialCommandGroup goStraightTurn = robotDrive.AutoCommandFactory(Trajectories.goStraightTurn);
    SequentialCommandGroup OneBucketAuto = new SequentialCommandGroup(
      pivotAutoOuttake,
      outtake2,
      goStraight);

   autoChooser.addOption("Nothing", null);
   autoChooser.addOption("Straight Auto", goStraight);
   autoChooser.addOption("Straight and Turn Auto", goStraightTurn);
   autoChooser.addOption("One Bucket Auto", OneBucketAuto);


  //  autoChooser = AutoBuilder.buildAutoChooser();
   
  SmartDashboard.putData("Auto Chooser", autoChooser);

  }
 
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

   // SmartDashboard.putData("Straight Auto", new PathPlannerAuto("Straight Auto"));
    //SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));



    



    
    //Intake bucket
    new JoystickButton(operatorController, Buttons.B).whileTrue(new IntakeControllerCmd(intakeSubsystem, 
      () -> IntakeConstants.kIntakeMotorSpeed, -1));

    //Outtake bucket
    new JoystickButton(operatorController, Buttons.A).whileTrue(new IntakeControllerCmd(intakeSubsystem, 
      () -> IntakeConstants.kIntakeMotorSpeed, 1));

      
    //Up elevator
    new JoystickButton(operatorController, Buttons.RB).whileTrue(new ElevatorControllerCmd(elevatorSubsystem, 
    () -> true, () -> false));

    //Down elevator
    new JoystickButton(operatorController, Buttons.LB).whileTrue(new ElevatorControllerCmd(elevatorSubsystem, 
    () -> false, () -> true));



    //Set pivot position to up right intake position
    new JoystickButton(operatorController, Buttons.X).whileTrue(new SetPivotCmd(pivotSubsystem, 0));
    
    //Set pivot position to knocked over intake position
    new JoystickButton(operatorController, Buttons.Y).whileTrue(new SetPivotCmd(pivotSubsystem, 1));

    //Set pivot position to scoring position

   

    //Elevator to middle position
    new POVButton(operatorController, Buttons.RIGHT_ARR).whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 0),new SetPivotCmd(pivotSubsystem, 0)));
    
    //Elevator to bottom position
    new POVButton(operatorController, Buttons.DOWN_ARR).whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 1),new SetPivotCmd(pivotSubsystem, 1)));

    //Elevator to top position
    new POVButton(operatorController, Buttons.UP_ARR).whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 2),new SetPivotCmd(pivotSubsystem, 5)));




    
    // //Reset pivot encoder at illegal position
    // new JoystickButton(operatorController, Buttons.Maria).whileTrue(new ResetPivotCmd(pivotSubsystem));








    //Rumble controllers
    //new JoystickButton(driverController, Buttons.Maria).whileTrue(new RumbleCmd(operatorController, 1, 1.00));
    new JoystickButton(operatorController, Buttons.L3).whileTrue(new RumbleCmd(driverController, 1, 1.00));
    new JoystickButton(operatorController, Buttons.R3).whileTrue(new RumbleCmd(driverController, 2, 1.00));



    //Zero Heading
    new JoystickButton(driverController, Buttons.X).toggleOnTrue(new AllForNaught(robotDrive));

    //Slow drive with d-pad
    new POVButton(driverController, Buttons.DOWN_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> -DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> 0.0, () -> true,  () -> false));
    new POVButton(driverController, Buttons.UP_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> 0.0, () -> true,  () -> false));
    new POVButton(driverController, Buttons.RIGHT_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> 0.0, () -> -DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> true,  () -> false));
    new POVButton(driverController, Buttons.LEFT_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> 0.0, () -> DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> true,  () -> false));



   // -----------------------------------------------new JoystickButton(driverController, Buttons.B).onTrue(new InstantCommand(() -> robotDrive.resetEncoders()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return the autonomous command given by the drop-down selector in ShuffleBoard
   return autoChooser.getSelected();
  }

}
