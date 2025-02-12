package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences.ELEVATOR;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.util.hardware.MotionControl.LinearController;
import frc.robot.util.hardware.MotionControl.RealLinearController;
import frc.robot.util.hardware.linear.LinearRatios;

/**
 * The Elevator subsystem controls the elevator mechanism of the robot. It extends the
 * DualLinearController to manage the elevator's movement.
 *
 * <p>Available Commands: - setHeightCommand(Distance height): Sets the elevator to a specific
 * height. - up(): Moves the elevator up by 1 inch. - down(): Moves the elevator down by 1 inch. -
 * coralL1(): Moves the elevator to the Coral Level 1 height. - coralL2(): Moves the elevator to the
 * Coral Level 2 height. - coralL3(): Moves the elevator to the Coral Level 3 height. - coralL4():
 * Moves the elevator to the Coral Level 4 height. - coralIntake(): Moves the elevator to the Coral
 * Intake height. - algaeGround(): Moves the elevator to the Algae Ground height. - algaeL2(): Moves
 * the elevator to the Algae Level 2 height. - algaeL3(): Moves the elevator to the Algae Level 3
 * height. - algaeBarge(): Moves the elevator to the Algae Barge height. - algaeProcessor(): Moves
 * the elevator to the Algae Processor height. - stow(): Moves the elevator to the stow height.
 *
 * <p>The subsystem also includes methods to run and stop the elevator motors, and to handle
 * periodic updates and simulation-specific behavior.
 */
public class Elevator extends SubsystemBase {
  private LinearController linearController;

  public Elevator() {
    linearController = LinearController.get(
        "Elevator/LinearController",
        CAN.ELEVATOR_LEFT,
        CAN.ELEVATOR_RIGHT,
        DIO.ELEVATOR_ENCODER,
        Constants.ELEVATOR.ENCODER_OFFSET.in(Rotations), // CHANGE THIS
        new LinearRatios(60.0, Inches.of(0.75), 3),
        ELEVATOR.MIN_HEIGHT,
        ELEVATOR.MAX_HEIGHT,
        Inches.of(0.5),
        () -> ENABLED_SYSTEMS.ELEVATOR && RobotContainer.getVoltage() >= VOLTAGE_LADDER.ELEVATOR,
        new RealLinearController.ControlConstants(1.0, 0.0, 0.0));

    // setDefaultCommand(Commands.run(this::stopMotors, this));
  }

  public Command setHeightCommand(Distance height) {
    return run(() -> linearController.setTargetHeight(height)).until(linearController::doneMoving);
  }

  public Command up() {
    return Commands.runEnd(linearController::moveUp, linearController::stopMotors, this);
  }

  public Command down() {
    return Commands.runEnd(linearController::moveDown, linearController::stopMotors, this);
  }

  public Command coralL1() {
    return setHeightCommand(ELEVATOR.CORAL.L1_HEIGHT);
  }

  public Command coralL2() {
    return setHeightCommand(ELEVATOR.CORAL.L2_HEIGHT);
  }

  public Command coralL3() {
    return setHeightCommand(ELEVATOR.CORAL.L3_HEIGHT);
  }

  public Command coralL4() {
    return setHeightCommand(ELEVATOR.CORAL.L4_HEIGHT);
  }

  public Command coralIntake() {
    return setHeightCommand(ELEVATOR.CORAL.INTAKE_HEIGHT);
  }

  public Command algaeGround() {
    return setHeightCommand(ELEVATOR.ALGAE.GROUND_HEIGHT);
  }

  public Command algaeL2() {
    return setHeightCommand(ELEVATOR.ALGAE.L2_HEIGHT);
  }

  public Command algaeL3() {
    return setHeightCommand(ELEVATOR.ALGAE.L3_HEIGHT);
  }

  public Command algaeBarge() {
    return setHeightCommand(ELEVATOR.ALGAE.BARGE_HEIGHT);
  }

  public Command algaeProcessor() {
    return setHeightCommand(ELEVATOR.ALGAE.PROCESSOR_HEIGHT);
  }

  public Command stow() {
    return setHeightCommand(ELEVATOR.STOW_HEIGHT);
  }

  public Command stop() {
    return run(linearController::stopMotors);
  }

  public Command test() {
    return Commands.sequence(stow(), coralL1(), coralL3(), algaeBarge(), stow());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
