package frc.robot.commands;

import com.team6962.lib.utils.CommandUtils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;

public class PieceCombos {
  Elevator elevator;
  Manipulator manipulator;

  public PieceCombos(Elevator elevator, Manipulator manipulator) {
    this.elevator = elevator;
    this.manipulator = manipulator;
  }

  public Command intakeCoral() {
    return manipulator
        .pivot
        .safe()
        .andThen(elevator.coralIntake())
        .andThen(manipulator.intakeCoral());
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
    return manipulator.pivot.safe().andThen(elevator.coralL1()).andThen(manipulator.placeCoralL1());
  }

  public Command coralL2() {
    return manipulator
        .pivot
        .safe()
        .andThen(elevator.coralL2())
        .andThen(manipulator.placeCoralL23());
  }

  public Command coralL3() {
    return manipulator
        .pivot
        .safe()
        .andThen(elevator.coralL3())
        .andThen(manipulator.placeCoralL23());
  }

  public Command coralL4() {
    return manipulator.pivot.safe().andThen(elevator.coralL4()).andThen(manipulator.placeCoralL4());
  }

  public Command pickupGroundAlgae() {
    return manipulator
        .pivot
        .safe()
        .andThen(elevator.algaeGround())
        .andThen(manipulator.pickupGroundAlgae());
  }

  public Command algaeL2() {
    return manipulator
        .pivot
        .safe()
        .andThen(elevator.algaeL2())
        .andThen(manipulator.pickupReefAlgae());
  }

  public Command algaeL3() {
    return manipulator
        .pivot
        .safe()
        .andThen(elevator.algaeL3())
        .andThen(manipulator.pickupReefAlgae());
  }

  public Command algaeBarge() {
    return manipulator
        .pivot
        .safe()
        .andThen(elevator.algaeBarge())
        .andThen(manipulator.placeBargeAlgae());
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
