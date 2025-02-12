// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.team6962.lib.swerve.SwerveDrive;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.elevator.Elevator;
// import frc.robot.subsystems.manipulator.Manipulator;

// /** An example command that uses an example subsystem. */
// public class CommandQueue extends Command {
//   private final SwerveDrive swerveDrive;
//   private final Elevator elevator;
//   private final Manipulator manipulator;

//   private Command alignmentCommand;
//   private Command levelCommand;
//   private Command pieceCommand;

//   public CommandQueue(
//     SwerveDrive swerveDrive,
//     Elevator elevator,
//     Manipulator manipulator  
//   ) {
//     this.swerveDrive = swerveDrive;
//     this.elevator = elevator;
//     this.manipulator = manipulator;
//     addRequirements(swerveDrive, elevator, manipulator);
//   }

//   public void setAlignmentCommand(Command alignmentCommand) {
//     this.alignmentCommand = alignmentCommand;
//   }

//   public void setLevelCommand(Command levelCommand) {
//     this.levelCommand = levelCommand;
//   }

//   public void setPieceCommand(Command pieceCommand) {
//     this.pieceCommand = pieceCommand;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     Commands.parallel(
//       alignmentCommand,
//       levelCommand);
//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }

// //     3  4
// //   2      5     
// // 1          6  
// // 12         7   
// //   11     8
// //     10 9     