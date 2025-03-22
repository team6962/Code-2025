package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ELEVATOR;
import frc.robot.Constants.Constants.MANIPULATOR_PIVOT;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;

public class SafeSubsystems extends SubsystemBase {
  Manipulator manipulator;
  Elevator elevator;

  public SafeSubsystems(Elevator elevator, Manipulator manipulator) {
    this.elevator = elevator;
    this.manipulator = manipulator;
  }

  public static Angle calcSafeMinAngle(Distance elevatorHeight) {
    Angle returnAngle = MANIPULATOR_PIVOT.MIN_ANGLES.ceilingEntry(elevatorHeight).getValue();
    if (returnAngle == null) {
      return MANIPULATOR_PIVOT.SAFE_ANGLE;
    }

    return returnAngle;
  }

  public static Angle calcSafeMaxAngle(Distance elevatorHeight) {
    Angle returnAngle = MANIPULATOR_PIVOT.MAX_ANGLES.ceilingEntry(elevatorHeight).getValue();
    if (returnAngle == null) {
      return MANIPULATOR_PIVOT.SAFE_ANGLE;
    }
    return returnAngle;
  }

  public Command safeMoveCommand(Command elevatorCommand, Command manipulatorCommand, Distance targetHeight) {
    Command safeMoveCommand =
         new ConditionalCommand(
             Commands.none(),
             manipulator.pivot.safe(),
             () -> (targetHeight.minus(elevator.getAverageHeight()).abs(Inches) < ELEVATOR.TOLERANCE.abs(Inches)));
    Command safeElevatorCommand = 
         new ConditionalCommand(
              elevator.coralL1(),
              Commands.none(), 
              () -> targetHeight.lte(ELEVATOR.BASE_HEIGHT) && elevator.getAverageHeight().gt(ELEVATOR.CORAL.L3_HEIGHT))
          .andThen(elevatorCommand);
     return Commands.sequence(safeMoveCommand, safeElevatorCommand, manipulatorCommand);
  }

  public Command parallelSafeCommand(Command elevatorCommand, Command manipulatorCommand) {
    if (Constants.SAFETIES_ENABLED) {
      return Commands.sequence(
          manipulator.pivot.safe(), Commands.parallel(elevatorCommand, manipulatorCommand));
    }

    return Commands.parallel(elevatorCommand, manipulatorCommand);
  }

  @Override
  public void periodic() {
    if (Constants.SAFETIES_ENABLED) {
      Distance elevatorHeight = elevator.getAverageHeight();
      Angle minAngle = calcSafeMinAngle(elevatorHeight);
      Angle maxAngle = calcSafeMaxAngle(elevatorHeight);

      manipulator.pivot.setMinMaxAngle(minAngle, maxAngle);
    }
  }
}
