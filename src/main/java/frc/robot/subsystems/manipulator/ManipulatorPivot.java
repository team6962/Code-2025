package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.MANIPULATOR_PIVOT;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.util.hardware.MotionControl.PivotController;

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
      Preferences.MANIPULATOR_PIVOT.MIN_LOW_ANGLE,
      Preferences.MANIPULATOR_PIVOT.MAX_ANGLE,
      Degrees.of(0.25),
      false);
    // setDefaultCommand(stow());
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
    return this.run(() -> setAngle(angleSupplier.get())).until(this::doneMoving);
  }

  public Command intakeCoral() {
    return pivotTo(() -> Preferences.MANIPULATOR_PIVOT.CORAL.INTAKE_ANGLE);
  }

  public Command coralL23() {
    return pivotTo(() -> Preferences.MANIPULATOR_PIVOT.CORAL.L23_ANGLE);
  }

  public Command coralL4() {
    return pivotTo(() -> Preferences.MANIPULATOR_PIVOT.CORAL.L4_ANGLE);
  }

  public Command algaeReef() {
    return pivotTo(() -> Preferences.MANIPULATOR_PIVOT.ALGAE.REEF_ANGLE);
  }

  public Command algaeBarge() {
    return pivotTo(() -> Preferences.MANIPULATOR_PIVOT.ALGAE.BARGE_ANGLE);
  }

  public Command algaeProcessor() {
    return pivotTo(() -> Preferences.MANIPULATOR_PIVOT.ALGAE.PROCESSOR_ANGLE);
  }

  public Command algaeGround() {
    return pivotTo(() -> Preferences.MANIPULATOR_PIVOT.ALGAE.GROUND_ANGLE);
  }

  public Command stow() {
    return pivotTo(() -> Preferences.MANIPULATOR_PIVOT.STOW_ANGLE);
  }

  public Command stop() {
    return Commands.run(this::stopMotor);
  }

  public Command up() {
    return Commands.runEnd(this::moveUp, this::stopMotor);
  }

  public Command down() {
    return Commands.runEnd(this::moveDown, this::stopMotor);
  }
}
