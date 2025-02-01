package frc.robot.subsystems.hang;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.HANG;
import frc.robot.Constants.Preferences;
import frc.robot.util.hardware.MotionControl.PivotController;
import frc.robot.util.hardware.SparkMaxUtil;

public class Hang extends SubsystemBase {
  private State state = State.OFF;
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private DutyCycleEncoder hangEncoder;
  private PivotController controller;

  public Hang() {
    motor = new SparkMax(CAN.HANG, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
    SparkMaxUtil.saveAndLog(this, motor, motorConfig);
    controller =
        new PivotController(
            this,
            motor,
            DIO.HANG_ENCODER,
            0,
            0,
            0,
            0,
            Preferences.HANG_PIVOT.MIN_ANGLE,
            Preferences.HANG_PIVOT.MAX_ANGLE,
            Degrees.of(2.0),
            false);
  }

  public enum State {
    OFF,
    CLIMB,
    REVERSE
  }

  public Command setState(State state) {
    return runEnd(() -> this.state = state, () -> this.state = State.OFF);
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_HANG) {
      return;
    }

    if (RobotState.isDisabled()) {
      state = State.OFF;
    }

    double motorPower = 0.0;

    switch (state) {
      case REVERSE:
        controller.setTargetAngle(Preferences.HANG_PIVOT.STOW_ANGLE);
        controller.run();
        break;
      case CLIMB:
        controller.setTargetAngle(Preferences.HANG_PIVOT.HANG_ANGLE);
        controller.run();
        break;
      case OFF:
        break;
    }

    if ((hangEncoder.get() >= HANG.EXTEND_HEIGHT && motorPower > 0.0)
        || (hangEncoder.get() <= HANG.RETRACT_HEIGHT && motorPower < 0.0)) {
      motorPower = 0.0;
    }

    // Makes sure we dont overshoot our limits for left hang arm

  }

  @Override
  public void simulationPeriodic() {}
}
