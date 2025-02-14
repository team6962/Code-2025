package frc.robot.subsystems.intake.pivot;

import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import java.util.function.Supplier;

public abstract class IntakePivot extends SubsystemBase {
  public abstract Command setAngle(Supplier<Angle> angle);

  public IntakePivot() {
    Logger.logMeasure("IntakePivot/targetAngle", this::getTargetAngle);
    Logger.logMeasure("IntakePivot/measuredAngle", this::getMeasuredAngle);
    Logger.logMeasure("IntakePivot/appliedVoltage", this::getAppliedVoltage);
  }

  public Command lower() {
    return setAngle(() -> Preferences.INTAKE.PIVOT_DOWN);
  }

  public Command raise() {
    return setAngle(() -> Preferences.INTAKE.PIVOT_UP);
  }

  public static IntakePivot create() {
    if (RobotBase.isReal() && ENABLED_SYSTEMS.INTAKE) {
      return new RealIntakePivot();
    } else {
      return new SimIntakePivot();
    }
  }

  protected abstract Angle getTargetAngle();

  protected abstract Angle getMeasuredAngle();

  protected abstract Voltage getAppliedVoltage();
}
