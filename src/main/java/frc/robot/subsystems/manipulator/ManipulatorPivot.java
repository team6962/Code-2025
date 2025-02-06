package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.MANIPULATOR_PIVOT;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.RobotContainer;
import frc.robot.util.hardware.MotionControl.PivotController;
import java.util.function.Supplier;

public class ManipulatorPivot extends SubsystemBase {
  private SparkMax motor;
  private PivotController controller;
  private boolean isCalibrating = false;

  public ManipulatorPivot() {
    motor = new SparkMax(CAN.MANIPULATOR_PIVOT, MotorType.kBrushless);
    controller =
        new PivotController(
            this,
            motor,
            DIO.MANIPULATOR_ENCODER,
            MANIPULATOR_PIVOT.ABSOLUTE_POSITION_OFFSET.in(Rotations),
            MANIPULATOR_PIVOT.PROFILE.kP,
            MANIPULATOR_PIVOT.PROFILE.kS,
            MANIPULATOR_PIVOT.GEARING,
            Preferences.MANIPULATOR_PIVOT.MIN_ANGLE,
            Preferences.MANIPULATOR_PIVOT.MAX_ANGLE,
            Degrees.of(0.25),
            true);

    setDefaultCommand(stow());
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.MANIPULATOR) return;
    if (isCalibrating) return;
    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.MANIPULATOR) {
      motor.stopMotor();
      return;
    }
    if (RobotState.isDisabled()) {
      controller.setTargetAngle(controller.getPosition());
    }

    controller.run();
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
    return Commands.run(controller::stop, this);
  }

  public Command pivotTo(Supplier<Angle> angleSupplier) {
    return Commands.run(() -> controller.setTargetAngle(angleSupplier.get()), this)
        .until(this::doneMoving);
  }

  public boolean isInRange(Angle angle) {
    if (angle == null) return false;
    return controller.isInRange(angle);
  }

  public Angle getPosition() {
    return controller.getPosition();
  }

  public boolean doneMoving() {
    return controller.doneMoving();
  }
}
