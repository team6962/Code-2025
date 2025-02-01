package frc.robot.util.hardware;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.StatusChecks;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Constants.NEO;

public final class TalonFXUtil {
  public static void configure(
      TalonFXConfiguration config, InvertedValue inverted, NeutralModeValue neutralMode) {
    configure(config, inverted, neutralMode, NEO.SAFE_FREE_CURRENT, NEO.SAFE_STALL_CURRENT);
  }

  public static void configure(
      TalonFXConfiguration config,
      InvertedValue inverted,
      NeutralModeValue neutralMode,
      int supplyCurrentLimit,
      int statorCurrentLimit) {
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;

    config.MotorOutput.NeutralMode = neutralMode;
    config.MotorOutput.Inverted = inverted;

    // config.voltageCompensation(12.0);
    // config.openLoopRampRate(NEO.SAFE_RAMP_RATE);
  }

  public static void configureCANStatusFrames(
      SparkMax motor, boolean velocityTemperatureVoltageCurrent, boolean position) {
    configureCANStatusFrames(
        motor, velocityTemperatureVoltageCurrent, position, false, false, false, false);
  }

  public static void configureCANStatusFrames(
      SparkMax motor,
      boolean velocityTemperatureVoltageCurrent,
      boolean position,
      boolean analogSensor,
      boolean alternateEncoder,
      boolean absoluteEncoderPosition,
      boolean absoluteEncoderVelocity) {

    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10), motor);
    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1,
    // velocityTemperatureVoltageCurrent ? 20 : 250), motor);
    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, position ? 250 : 250),
    // motor);
    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, analogSensor ? 20 :
    // 250), motor);
    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, alternateEncoder ? 20 :
    // 250), motor);
    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, absoluteEncoderPosition
    // ? 20 : 250), motor);
    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, absoluteEncoderVelocity
    // ? 20 : 250), motor);
    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  }

  public static void configureEncoder(SparkMaxConfig config, double encoderConversionFactor) {
    config.encoder.positionConversionFactor(encoderConversionFactor);
    config.encoder.velocityConversionFactor(encoderConversionFactor);
  }

  public static void saveAndLog(Subsystem subsystem, TalonFX motor, TalonFXConfiguration config) {
    motor.getConfigurator().apply(config);

    // Logger.autoLog(subsystem, logPath + "current",          () -> motor.getOutputCurrent());
    // Logger.autoLog(subsystem, logPath + "voltage",          () -> motor.getBusVoltage());
    // Logger.autoLog(subsystem, logPath + "setOutput",        () -> motor.get());
    // Logger.autoLog(subsystem, logPath + "appliedOutput",    () -> motor.getAppliedOutput());
    // Logger.autoLog(subsystem, logPath + "appliedVoltage",   () -> motor.getBusVoltage() *
    // motor.getAppliedOutput());
    // Logger.autoLog(subsystem, logPath + "powerWatts",       () -> motor.getBusVoltage() *
    // motor.getAppliedOutput() * motor.getOutputCurrent());
    // Logger.autoLog(subsystem, logPath + "motorTemperature", () -> motor.getMotorTemperature());
    // Logger.autoLog(subsystem, logPath + "position",         () -> encoder.getPosition());
    // Logger.autoLog(subsystem, logPath + "velocity",         () -> encoder.getVelocity());

    StatusChecks.under(subsystem).add("TalonFX", motor);
    // StatusChecks.addCheck(subsystem, logPath + "isTooHot", () -> motor.getMotorTemperature() <=
    // NEO.SAFE_TEMPERATURE);
    // configure(() -> motor.burnFlash(), motor);
  }
}
