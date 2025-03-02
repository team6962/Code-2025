package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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

import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.MeasureMath;

public class ManipulatorPivot extends PivotController {

  public ManipulatorPivot() {
    super(
        "Manipulator Pivot",
        CAN.MANIPULATOR_PIVOT,
        DIO.MANIPULATOR_ENCODER,
        MANIPULATOR_PIVOT.ABSOLUTE_POSITION_OFFSET.in(Rotations),
        MANIPULATOR_PIVOT.PROFILE.kP,
        MANIPULATOR_PIVOT.PROFILE.kI,
        MANIPULATOR_PIVOT.PROFILE.kD,
        MANIPULATOR_PIVOT.PROFILE.kS,
        MANIPULATOR_PIVOT.GEARING,
        MANIPULATOR_PIVOT.MIN_ANGLE,
        MANIPULATOR_PIVOT.MAX_ANGLE,
        MANIPULATOR_PIVOT.TOLERANCE,
        false);
    // setDefaultCommand(stow());

    // setDefaultCommand(pivotTo(() -> stopAngle));

    setDefaultCommand(hold());
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.MANIPULATOR) return;
    super.periodic();
    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.MANIPULATOR) {
      stopMotor();
      return;
    }
  }

  public Command pivotTo(Supplier<Angle> angleSupplier, Angle tolerance) {
    if (!ENABLED_SYSTEMS.MANIPULATOR) return stop();
    return run(() -> moveTowards(angleSupplier.get())).until(() -> this.doneMoving(tolerance)).finallyDo(this::seedEncoder);
  }

  public Command pivotTo(Supplier<Angle> angleSupplier) {
    if (!ENABLED_SYSTEMS.MANIPULATOR) return stop();
    return run(() -> moveTowards(angleSupplier.get())).until(this::doneMoving).finallyDo(this::seedEncoder);
  }

  public Command hold() {
    return Commands.defer(
        () -> {
          Angle position = getRelativePosition();

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

  public Command safe(Angle tolerance) {
    // Angle currentAngle = getAbsolutePosition();
    // if (currentAngle.lt(MANIPULATOR_PIVOT.SAFE_MIN_ANGLE)) {
    //     Logger.log("GRUB", "below" + Timer.getFPGATimestamp());
    //     return pivotTo(() -> MANIPULATOR_PIVOT.SAFE_MIN_ANGLE, tolerance);
    // }
    // if (currentAngle.gt(MANIPULATOR_PIVOT.SAFE_MAX_ANGLE)) {
    //     Logger.log("GRUB", "above" + Timer.getFPGATimestamp());
    //     return pivotTo(() -> MANIPULATOR_PIVOT.SAFE_MAX_ANGLE, tolerance);
    // }
    // Logger.log("GRUB", "within" + Timer.getFPGATimestamp());
    // return pivotTo(() -> MANIPULATOR_PIVOT.SAFE_ANGLE, tolerance);

    Angle targetAngle = MeasureMath.clamp(
      getAbsolutePosition(),
      MANIPULATOR_PIVOT.SAFE_MIN_ANGLE,
      MANIPULATOR_PIVOT.SAFE_MAX_ANGLE
    );

    return pivotTo(() -> targetAngle, tolerance);
  }

  public Command safe() {
    return safe(MANIPULATOR_PIVOT.SAFE_TOLERANCE);
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
            .angularPosition(getRelativePosition())
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
