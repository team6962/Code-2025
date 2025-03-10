// package frc.robot.commands.drive;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.elevator.Elevator;
// import frc.robot.subsystems.manipulator.Manipulator;

// public class OperatorFineControls extends Command {
//   private Elevator elevator;
//   private XboxController controller;
//   private Manipulator manipulator;

//   public OperatorFineControls(
//       Elevator elevator, Manipulator manipulator, XboxController controller) {
//     this.elevator = elevator;
//     this.controller = controller;
//     this.manipulator = manipulator;
//   }

//   @Override
//   public void execute() {
//     double elevatorSpeed = controller.getLeftY() * 0.2;

//     if (elevator.getCurrentCommand() == elevator.getDefaultCommand()
//         && Math.abs(controller.getLeftY()) > 0.1) {
//       elevator.move(elevatorSpeed).schedule();
//     }

//     double manipulatorSpeed = controller.getRightY() * 0.2;

//     if (manipulator.pivot.getCurrentCommand() == manipulator.pivot.getDefaultCommand()
//         && Math.abs(controller.getRightY()) > 0.1) {
//       manipulator.pivot.move(manipulatorSpeed).schedule();
//     }
//   }
// }
