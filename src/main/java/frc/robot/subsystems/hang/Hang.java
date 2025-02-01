package frc.robot.subsystems.hang;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import frc.robot.util.hardware.MotionControl.PivotController;

public class Hang extends SubsystemBase {
  private SparkMax motor;
  private PivotController controller;

  public Hang() {
    motor = new SparkMax(CAN.HANG, MotorType.kBrushless);
    controller =
        new PivotController(
            this,
            motor,
            DIO.HANG_ENCODER,
            Constants.HANG_PIVOT.ENCODER_OFFSET,
            Constants.HANG_PIVOT.PROFILE.kP,
            Constants.HANG_PIVOT.PROFILE.kS,
            Constants.HANG_PIVOT.GEARING,
            Preferences.HANG_PIVOT.MIN_ANGLE,
            Preferences.HANG_PIVOT.MAX_ANGLE,
            Degrees.of(2.0),
            false);
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.HANG) controller.stop();
  }

  public Command deploy() {
    return setTargetAngleCommand(Preferences.HANG_PIVOT.HANG_ANGLE);
  }

  public Command stow() {
    return setTargetAngleCommand(Preferences.HANG_PIVOT.STOW_ANGLE);
  }

  public Command setTargetAngleCommand(Angle angle) {
    return run(() -> setTargetAngleAndRun(angle)).until(this::doneMoving);
  }

  public void setTargetAngle(Angle angle) {
    controller.setTargetAngle(angle);
  }

  public void setTargetAngleAndRun(Angle angle) {
    controller.setTargetAngle(angle);
    controller.run();
  }

  public boolean doneMoving() {
    return controller.doneMoving();
  }

  @Override
  public void simulationPeriodic() {}
}
