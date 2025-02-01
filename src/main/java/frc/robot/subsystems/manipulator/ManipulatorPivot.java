package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Degrees;

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
            DIO.MANIPULATOR_PIVOT,
            MANIPULATOR_PIVOT.ABSOLUTE_POSITION_OFFSET,
            MANIPULATOR_PIVOT.PROFILE.kP,
            MANIPULATOR_PIVOT.PROFILE.kS,
            MANIPULATOR_PIVOT.GEARING,
            Preferences.MANIPULATOR_PIVOT.MIN_ANGLE,
            Preferences.MANIPULATOR_PIVOT.MAX_ANGLE,
            Degrees.of(0.25),
            true);

    setDefaultCommand(pivotTo(() -> Preferences.MANIPULATOR_PIVOT.MIN_ANGLE));
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.SHOOTER) return;
    if (isCalibrating) return;
    if (RobotState.isDisabled()) {
      controller.setTargetAngle(controller.getPosition());
    }

    controller.run();

    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.SHOOTER) motor.stopMotor();
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
