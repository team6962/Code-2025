package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.MANIPULATOR_PIVOT;
import frc.robot.Constants.Constants.VOLTAGE_LADDER;
import frc.robot.RobotContainer;
import frc.robot.util.hardware.motion.PivotController;
import java.util.Set;
import java.util.function.Supplier;

public class ManipulatorPivot extends PivotController {
  private boolean isCalibrating = false;

  public ManipulatorPivot() {
    super(
        CAN.MANIPULATOR_PIVOT,
        DIO.MANIPULATOR_ENCODER,
        MANIPULATOR_PIVOT.ABSOLUTE_POSITION_OFFSET.in(Rotations),
        MANIPULATOR_PIVOT.PROFILE.kP,
        MANIPULATOR_PIVOT.PROFILE.kI,
        MANIPULATOR_PIVOT.PROFILE.kD,
        MANIPULATOR_PIVOT.PROFILE.kS,
        MANIPULATOR_PIVOT.GEARING,
        MANIPULATOR_PIVOT.MIN_LOW_ANGLE,
        MANIPULATOR_PIVOT.MAX_ANGLE,
        MANIPULATOR_PIVOT.TOLERANCE,
        false);
    // setDefaultCommand(stow());

    // setDefaultCommand(pivotTo(() -> stopAngle));

    // setDefaultCommand(hold());
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.MANIPULATOR) return;
    if (isCalibrating) return;
    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.MANIPULATOR) {
      stopMotor();
      return;
    }
  }

  public Command pivotTo(Supplier<Angle> angleSupplier) {
    if (!ENABLED_SYSTEMS.MANIPULATOR) return stop();
    return run(() -> moveTowards(angleSupplier.get())).until(this::doneMoving);
  }

  public Command hold() {
    return Commands.defer(
        () -> {
          Angle position = getPosition();

          return run(() -> moveTowards(position));
        },
        Set.of(this));
  }

  public Command coralIntake() {
    return pivotTo(() -> MANIPULATOR_PIVOT.CORAL.INTAKE_ANGLE);
  }

  public Command coralL1() {
    return pivotTo(() -> MANIPULATOR_PIVOT.CORAL.L1_ANGLE);
  }

  public Command coralL23() {
    return pivotTo(() -> MANIPULATOR_PIVOT.CORAL.L23_ANGLE);
  }

  public Command coralL4() {
    return pivotTo(() -> MANIPULATOR_PIVOT.CORAL.L4_ANGLE);
  }

  public Command algaeReef() {
    return pivotTo(() -> MANIPULATOR_PIVOT.ALGAE.REEF_ANGLE);
  }

  public Command algaeBarge() {
    return pivotTo(() -> MANIPULATOR_PIVOT.ALGAE.BARGE_ANGLE);
  }

  public Command algaeProcessor() {
    return pivotTo(() -> MANIPULATOR_PIVOT.ALGAE.PROCESSOR_ANGLE);
  }

  public Command algaeGround() {
    return pivotTo(() -> MANIPULATOR_PIVOT.ALGAE.GROUND_ANGLE);
  }

  public Command stow() {
    return pivotTo(() -> MANIPULATOR_PIVOT.STOW_ANGLE);
  }

  public Command safe() {
    return pivotTo(() -> MANIPULATOR_PIVOT.SAFE_ANGLE);
  }

  public Command pidTestMin() {
    return pivotTo(() -> MANIPULATOR_PIVOT.PID_MIN_ANGLE);
  }

  public Command pidTestMid() {
    return pivotTo(() -> MANIPULATOR_PIVOT.PID_MID_ANGLE);
  }

  public Command pidTestMax() {
    return pivotTo(() -> MANIPULATOR_PIVOT.PID_MAX_ANGLE);
  }

  public Command stop() {
    return run(this::stopMotor);
  }

  // public Command up() {
  //   return run(this::moveUp);
  // }

  // public Command down() {
  //   return run(this::moveDown);
  // }
  

  public Command calibrate() {
    SysIdRoutine calibrationRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.per(Second).of(4.0), Volts.of(6.0), Seconds.of(1.5)),
      new SysIdRoutine.Mechanism(
        voltage -> {
          if (!canMoveInDirection(voltage.in(Volts))) {
            DriverStation.reportError("Reached limit switch", false);

            return;
          }

          motor.setVoltage(voltage);
        },
        log -> log.motor("manipulator-pivot")
            .voltage(Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()))
            .angularPosition(getPosition())
            .angularVelocity(RotationsPerSecond.of(motor.getEncoder().getVelocity())),
        this));

    return Commands.sequence(
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(motor::stopMotor),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(motor::stopMotor),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(motor::stopMotor),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(motor::stopMotor),
      Commands.waitSeconds(1.0)
    );
  }
}
