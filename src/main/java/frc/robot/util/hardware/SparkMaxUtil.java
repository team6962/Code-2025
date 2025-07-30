package frc.robot.util.hardware;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants.NEO;
import java.util.function.Supplier;

public final class SparkMaxUtil {
  public static void configure(SparkMaxConfig config, boolean inverted, IdleMode idleMode) {
    configure(config, inverted, idleMode, NEO.SAFE_FREE_CURRENT, NEO.SAFE_STALL_CURRENT);
  }

  public static void configure(
      SparkMaxConfig config,
      boolean inverted,
      IdleMode idleMode,
      int freeCurrentLimit,
      int stallCurrentLimit) {
    config.idleMode(idleMode);
    config.voltageCompensation(12.0);
    config.smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    config.openLoopRampRate(NEO.SAFE_RAMP_RATE);
    config.inverted(inverted);
  }

  private static void configure(Supplier<REVLibError> config, SparkMax motor) {
    for (int i = 0; i < 5; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
    }
    DriverStation.reportError(
        "Failure configuring spark max "
            + motor.getDeviceId()
            + "\n"
            + "Error: "
            + config.get().toString(),
        false);
  }

  public static void configure550(SparkMaxConfig config, boolean inverted, IdleMode idleMode) {
    configure(config, inverted, idleMode, 10, 40);
  }

  public static void configurePID(
      SparkMaxConfig config,
      double kP,
      double kI,
      double kD,
      double kV,
      double minValue,
      double maxValue,
      boolean wrap) {
    config.closedLoop.p(kP).i(kI).d(kD);

    if (wrap) {
      config.closedLoop.positionWrappingEnabled(true);
      config.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);
    }
  }

  public static void configureEncoder(SparkMaxConfig config, double encoderConversionFactor) {
    config.encoder.positionConversionFactor(encoderConversionFactor);
    config.encoder.velocityConversionFactor(encoderConversionFactor);
  }

  public static void configureFollower(SparkMaxConfig config, SparkMax leader, boolean inverted) {
    config.follow(leader, inverted);
  }

  public static void saveAndLog(Subsystem subsystem, SparkMax motor, SparkMaxConfig config) {
    saveAndLog(subsystem.getName(), motor, config);
  }

  public static void saveAndLog(String name, SparkMax motor, SparkMaxConfig config) {
    configure(() -> motor.setCANTimeout(0), motor);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
