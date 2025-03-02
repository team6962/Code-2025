package frc.robot.commands;

import com.team6962.lib.utils.CommandUtils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;

public class PieceCombos {
  Elevator elevator;
  Manipulator manipulator;
  SafeSubsystems safeSubsystems;

  public PieceCombos(Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems) {
    this.elevator = elevator;
    this.manipulator = manipulator;
    this.safeSubsystems = safeSubsystems;
  }

  public Command intakeCoral() {
    return safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.intakeCoral());
  }

  public Command coral(int level) {
    return switch (level) {
      case 1 -> coralL1();
      case 2 -> coralL2();
      case 3 -> coralL3();
      case 4 -> coralL4();
      default -> CommandUtils.noneWithRequirements(elevator, manipulator);
    };
  }

  public Command coralL1() {
    return safeSubsystems.parallelSafeCommand(
      elevator.coralL1(),
      manipulator.placeCoralL1());
  }

  public Command coralL2() {
    return safeSubsystems.parallelSafeCommand(
        elevator.coralL2(),
        manipulator.placeCoralL23());
  }

  public Command coralL3() {
    return safeSubsystems.parallelSafeCommand(
        elevator.coralL3(),
        manipulator.placeCoralL23());
  }

  public Command coralL4() {
    return safeSubsystems.parallelSafeCommand(
        elevator.coralL4(),
        manipulator.placeCoralL4());
  }

  public Command pickupGroundAlgae() {
    return safeSubsystems.parallelSafeCommand(
        elevator.algaeGround(),
        manipulator.pickupGroundAlgae());
  }

  public Command algae(int level) {
    return switch (level) {
      case 2 -> algaeL2();
      case 3 -> algaeL3();
      default -> CommandUtils.noneWithRequirements(elevator, manipulator);
    };
  }

  public Command algaeL2() {
    return safeSubsystems.parallelSafeCommand(
        elevator.algaeL2(),
        manipulator.pivot.algaeReef());
  }

  public Command algaeL3() {
    return safeSubsystems.parallelSafeCommand(
        elevator.algaeL3(),
        manipulator.pivot.algaeReef());
  }

  public Command algaeBarge() {
    return safeSubsystems.safeMoveCommand(
        elevator.algaeBarge(),
        manipulator.pivot.algaeBarge());
  }

  public Command algaeProcessor() {
    return safeSubsystems.parallelSafeCommand(elevator.algaeProcessor(), manipulator.placeProcessorAlgae());
  }

  public Command stow() {
    return manipulator.pivot.safe().andThen(elevator.stow()).andThen(manipulator.stow());
  }
}
