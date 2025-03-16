package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import com.team6962.lib.utils.CommandUtils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.ELEVATOR;
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
    return safeSubsystems.safeMoveCommand(
        elevator.coralIntake(), manipulator.intakeCoral(), ELEVATOR.CORAL.INTAKE_HEIGHT);
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
        elevator.coralL1(), manipulator.placeCoralL1(), ELEVATOR.CORAL.L1_HEIGHT);
  }

  public Command coralL2() {
    return safeSubsystems.safeMoveCommand(
        elevator.coralL2(), manipulator.placeCoralL23(), ELEVATOR.CORAL.L2_HEIGHT);
  }

  public Command coralL3() {
    return safeSubsystems.safeMoveCommand(
        elevator.coralL3(), manipulator.placeCoralL23(), ELEVATOR.CORAL.L3_HEIGHT);
  }

  public Command coralL4() {
    return safeSubsystems.safeMoveCommand(
        elevator.coralL4(), manipulator.placeCoralL4(), ELEVATOR.CORAL.L4_HEIGHT);
  }

  public Command pickupGroundAlgae() {
    return safeSubsystems.safeMoveCommand(
        elevator.algaeGround(), manipulator.pickupGroundAlgae(), ELEVATOR.ALGAE.GROUND_HEIGHT);
  }

  public Command algae(int level) {
    return switch (level) {
      case 2 -> algaeL2();
      case 3 -> algaeL3();
      default -> CommandUtils.noneWithRequirements(elevator, manipulator);
    };
  }

  public Command algaeL2() {
    return safeSubsystems.safeMoveCommand(
        elevator.algaeL2(), manipulator.pivot.algaeReef(), ELEVATOR.ALGAE.L2_HEIGHT);
  }

  public Command algaeL3() {
    return safeSubsystems.safeMoveCommand(
        elevator.algaeL3(), manipulator.pivot.algaeReef(), ELEVATOR.ALGAE.L3_HEIGHT);
  }

  public Command algaeBarge() {
    return safeSubsystems.safeMoveCommand(
        elevator.algaeBarge(), manipulator.pivot.algaeBarge(), ELEVATOR.ALGAE.BARGE_HEIGHT);
  }

  public Command algaeProcessor() {
    return safeSubsystems.safeMoveCommand(
        elevator.algaeProcessor(),
        manipulator.placeProcessorAlgae(),
        ELEVATOR.ALGAE.PROCESSOR_HEIGHT);
  }

  public Command stow() {
    return manipulator.pivot.safe().andThen(elevator.stow()).andThen(manipulator.stow());
  }

  public Command safeRaise() {
    return manipulator.pivot.pivotTo(() -> Degrees.of(-25.0)).andThen(elevator.coralL1());
  }
}
