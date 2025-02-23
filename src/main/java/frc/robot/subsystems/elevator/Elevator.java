package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ELEVATOR;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.VOLTAGE_LADDER;
import frc.robot.util.hardware.motion.DualLinearActuator;

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
public class Elevator extends DualLinearActuator {
  public Elevator() {
    super(
        "Elevator",
        CAN.ELEVATOR_LEFT,
        CAN.ELEVATOR_RIGHT,
        DIO.ELEVATOR_ENCODER,
        DIO.ELEVATOR_CEIL_LIMIT,
        DIO.ELEVATOR_FLOOR_LIMIT,
        Constants.ELEVATOR.ENCODER_OFFSET.in(Rotations), // CHANGE THIS
        Constants.ELEVATOR.PROFILE.kP,
        Constants.ELEVATOR.PROFILE.kS,
        Constants.ELEVATOR.GEARING,
        Constants.ELEVATOR.CYCLE_HEIGHT,
        ELEVATOR.BASE_HEIGHT,
        ELEVATOR.MIN_HEIGHT,
        ELEVATOR.MAX_HEIGHT,
        Inches.of(2.5));

    setDefaultCommand(hold());
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ELEVATOR) stopMotors();
    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.ELEVATOR) stopMotors();
    super.periodic();
  }

  public Command setHeight(Distance height) {
    if (!ENABLED_SYSTEMS.ELEVATOR) return stop();
    return this.run(() -> moveTo(height)).until(this::doneMoving);
  }

  public Command up() {
    return this.runEnd(this::moveUp, this::stopMotors);
  }

  public Command down() {
    return this.runEnd(this::moveDown, this::stopMotors);
  }

  public Command stop() {
    return this.run(this::stopMotors);
  }

  public Command hold() {
    return Commands.defer(
        () -> {
          Distance position = getAverageHeight();

          return run(() -> setHeight(position));
        },
        Set.of(this));
  }

  public Command stow() {
    return setHeight(ELEVATOR.STOW_HEIGHT);
  }

  public Command coralL1() {
    return setHeight(ELEVATOR.CORAL.L1_HEIGHT);
  }

  public Command coralL2() {
    return setHeight(ELEVATOR.CORAL.L2_HEIGHT);
  }

  public Command coralL3() {
    return setHeight(ELEVATOR.CORAL.L3_HEIGHT);
  }

  public Command coralL4() {
    return setHeight(ELEVATOR.CORAL.L4_HEIGHT);
  }

  public Command coralIntake() {
    return setHeight(ELEVATOR.CORAL.INTAKE_HEIGHT);
  }

  public Command algaeGround() {
    return setHeight(ELEVATOR.ALGAE.GROUND_HEIGHT);
  }

  public Command algaeL2() {
    return setHeight(ELEVATOR.ALGAE.L2_HEIGHT);
  }

  public Command algaeL3() {
    return setHeight(ELEVATOR.ALGAE.L3_HEIGHT);
  }

  public Command algaeBarge() {
    return setHeight(ELEVATOR.ALGAE.BARGE_HEIGHT);
  }

  public Command algaeProcessor() {
    return setHeight(ELEVATOR.ALGAE.PROCESSOR_HEIGHT);
  }

  public Command rezeroAtBottom() {
    return this.run(this::unsafeMoveDown).until(this::triggeredFloorLimit);
  }

  public Command test() {
    return Commands.sequence(stow(), coralL1(), coralL3(), algaeBarge(), stow());
  }

  public Command calibrate() {
    SysIdRoutine calibrationRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.per(Second).of(4.0), Volts.of(6.0), Seconds.of(2.5)),
      new SysIdRoutine.Mechanism(
        voltage -> {
          if (!canMoveInDirection(voltage.in(Volts))) {
            DriverStation.reportError("Reached limit switch", false);

            return;
          }

          leftMotor.setVoltage(voltage);
          rightMotor.setVoltage(voltage);
        },
        log -> log.motor("elevator")
            .voltage(Volts.of((
              leftMotor.getAppliedOutput() * leftMotor.getBusVoltage() +
              rightMotor.getAppliedOutput() * rightMotor.getBusVoltage()) / 2.0))
            .linearPosition(getAverageHeight())
            .linearVelocity(MetersPerSecond.of((leftMotor.getEncoder().getVelocity() + rightMotor.getEncoder().getVelocity()) / 2.0)),
        this));

    return Commands.sequence(
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> { leftMotor.stopMotor(); rightMotor.stopMotor(); }),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> { leftMotor.stopMotor(); rightMotor.stopMotor(); }),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> { leftMotor.stopMotor(); rightMotor.stopMotor(); }),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> { leftMotor.stopMotor(); rightMotor.stopMotor(); }),
      Commands.waitSeconds(1.0)
    );
  }
}
