package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.temp.Constants;
import frc.robot.temp.Constants.CAN;
import frc.robot.temp.Constants.DIO;
import frc.robot.temp.Constants.ELEVATOR;
import frc.robot.temp.Constants.ENABLED_SYSTEMS;
import frc.robot.temp.Constants.VOLTAGE_LADDER;
import frc.robot.RobotContainer;
import frc.robot.util.hardware.motion.DualLinearActuator;
import java.util.Set;

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
public class RealElevator extends DualLinearActuator implements Elevator {
  public RealElevator() {
    super(
        "Elevator",
        CAN.ELEVATOR_LEFT,
        CAN.ELEVATOR_RIGHT,
        DIO.ELEVATOR_CEIL_LIMIT,
        DIO.ELEVATOR_FLOOR_LIMIT,
        Constants.ELEVATOR.PROFILE.kP,
        Constants.ELEVATOR.PROFILE.kS,
        Constants.ELEVATOR.GEARING,
        Constants.ELEVATOR.CYCLE_HEIGHT,
        ELEVATOR.BASE_HEIGHT,
        ELEVATOR.MIN_HEIGHT,
        ELEVATOR.MAX_HEIGHT,
        ELEVATOR.TOLERANCE);

    setDefaultCommand(hold());
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.isElevatorEnabled()) stopMotors();
    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.ELEVATOR) stopMotors();
    super.periodic();
  }

  @Override
  public Command setHeight(Distance height) {
    if (!ENABLED_SYSTEMS.isElevatorEnabled()) return stop();
    return this.run(() -> moveTo(height)).until(this::doneMoving);
  }

  @Override
  public Command stop() {
    return this.run(this::stopMotors);
  }

  @Override
  public Command hold() {
    return Commands.defer(
        () -> {
          Distance position = getAverageHeight();

          return this.run(() -> moveTo(position));
        },
        Set.of(this));
  }

  @Override
  public Command stow() {
    return setHeight(ELEVATOR.STOW_HEIGHT);
  }

  @Override
  public Command coralL1() {
    return setHeight(ELEVATOR.CORAL.L1_HEIGHT);
  }

  @Override
  public Command coralL2() {
    return setHeight(ELEVATOR.CORAL.L2_HEIGHT);
  }

  @Override
  public Command coralL3() {
    return setHeight(ELEVATOR.CORAL.L3_HEIGHT);
  }

  @Override
  public Command coralL4() {
    return setHeight(ELEVATOR.CORAL.L4_HEIGHT);
  }

  @Override
  public Command coralIntake() {
    return setHeight(ELEVATOR.CORAL.INTAKE_HEIGHT);
  }

  @Override
  public Command algaeGround() {
    return setHeight(ELEVATOR.ALGAE.GROUND_HEIGHT);
  }

  @Override
  public Command algaeL2() {
    return setHeight(ELEVATOR.ALGAE.L2_HEIGHT);
  }

  @Override
  public Command algaeL3() {
    return setHeight(ELEVATOR.ALGAE.L3_HEIGHT);
  }

  @Override
  public Command algaeBarge() {
    return setHeight(ELEVATOR.ALGAE.BARGE_HEIGHT);
  }

  @Override
  public Command algaeProcessor() {
    return setHeight(ELEVATOR.ALGAE.PROCESSOR_HEIGHT);
  }

  @Override
  public Command ready() {
    return setHeight(ELEVATOR.AUTO.READY_HEIGHT);
  }

  @Override
  public Command rezeroAtBottom() {
    return this.run(this::unsafeMoveDown).until(this::triggeredFloorLimit);
  }

  public Command test() {
    return Commands.sequence(stow(), coralL1(), coralL3(), algaeBarge(), stow());
  }

  public Command calibrate() {
    SysIdRoutine calibrationRoutine =
        new SysIdRoutine(
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
                log ->
                    log.motor("elevator")
                        .voltage(
                            Volts.of(
                                (leftMotor.getAppliedOutput() * leftMotor.getBusVoltage()
                                        + rightMotor.getAppliedOutput()
                                            * rightMotor.getBusVoltage())
                                    / 2.0))
                        .linearPosition(getAverageHeight())
                        .linearVelocity(
                            MetersPerSecond.of(
                                (leftMotor.getEncoder().getVelocity()
                                        + rightMotor.getEncoder().getVelocity())
                                    / 2.0)),
                this));

    return Commands.sequence(
        Commands.waitSeconds(1.0),
        calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.runOnce(
            () -> {
              leftMotor.stopMotor();
              rightMotor.stopMotor();
            }),
        Commands.waitSeconds(1.0),
        calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.runOnce(
            () -> {
              leftMotor.stopMotor();
              rightMotor.stopMotor();
            }),
        Commands.waitSeconds(1.0),
        calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.runOnce(
            () -> {
              leftMotor.stopMotor();
              rightMotor.stopMotor();
            }),
        Commands.waitSeconds(1.0),
        calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
        Commands.runOnce(
            () -> {
              leftMotor.stopMotor();
              rightMotor.stopMotor();
            }),
        Commands.waitSeconds(1.0));
  }
}
