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
    return safeSubsystems.safeMoveCommand(
      elevator.coralL1(),
      manipulator.placeCoralL1());
  }

  public Command coralL2() {
    return safeSubsystems.safeMoveCommand(
        elevator.coralL2(),
        manipulator.placeCoralL23());
  }

  public Command coralL3() {
    return safeSubsystems.safeMoveCommand(
        elevator.coralL3(),
        manipulator.placeCoralL23());
  }

  public Command coralL4() {
    return safeSubsystems.safeMoveCommand(
        elevator.coralL4(),
        manipulator.placeCoralL4());
  }

  public Command pickupGroundAlgae() {
    return safeSubsystems.safeMoveCommand(
        elevator.algaeGround(),
        manipulator.pickupGroundAlgae());
  }

  public Command algaeL2() {
    return safeSubsystems.safeMoveCommand(
        elevator.algaeL2(),
        manipulator.pickupReefAlgae());
  }

  public Command algaeL3() {
    return safeSubsystems.safeMoveCommand(
        elevator.algaeL3(),
        manipulator.pickupReefAlgae());
  }

  public Command algaeBarge() {
    return safeSubsystems.safeMoveCommand(
        elevator.algaeBarge(),
        manipulator.pivot.algaeBarge());
  }

  public Command algaeProcessor() {
    return manipulator
        .pivot
        .safe()
        .andThen(elevator.algaeProcessor())
        .andThen(manipulator.pivot.algaeProcessor());
  }

  public Command stow() {
    return manipulator.pivot.safe().andThen(elevator.stow()).andThen(manipulator.stow());
  }
}
