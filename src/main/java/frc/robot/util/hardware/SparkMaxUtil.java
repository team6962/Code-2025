package frc.robot.util.hardware;
import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.StatusChecks;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Constants.NEO;

public final class SparkMaxUtil {
  public static void configure(SparkMaxConfig config, boolean inverted, IdleMode idleMode) {
    configure(config, inverted, idleMode, NEO.SAFE_FREE_CURRENT, NEO.SAFE_STALL_CURRENT);
  }

  public static void configure(SparkMaxConfig config, boolean inverted, IdleMode idleMode, int freeCurrentLimit, int stallCurrentLimit) {
    config.idleMode(idleMode);
    config.voltageCompensation(12.0);
    config.smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    config.openLoopRampRate(NEO.SAFE_RAMP_RATE);
    config.inverted(inverted);

    // String logPath = "motor" + motor.getDeviceId() + "/";

    // Logger.autoLog(subsystem, logPath + "current",          () -> motor.getOutputCurrent());
    // Logger.autoLog(subsystem, logPath + "voltage",          () -> motor.getBusVoltage());
    // Logger.autoLog(subsystem, logPath + "setOutput",        () -> motor.get());
    // Logger.autoLog(subsystem, logPath + "appliedOutput",    () -> motor.getAppliedOutput());
    // Logger.autoLog(subsystem, logPath + "appliedVoltage",   () -> motor.getBusVoltage() * motor.getAppliedOutput());
    // Logger.autoLog(subsystem, logPath + "powerWatts",       () -> motor.getBusVoltage() * motor.getAppliedOutput() * motor.getOutputCurrent());
    // Logger.autoLog(subsystem, logPath + "motorTemperature", () -> motor.getMotorTemperature());
    // Logger.autoLog(subsystem, logPath + "position",         () -> encoder.getPosition());
    // Logger.autoLog(subsystem, logPath + "velocity",         () -> encoder.getVelocity());

    // StatusChecks.addCheck(subsystem, logPath + "isTooHot", () -> motor.getMotorTemperature() <= NEO.SAFE_TEMPERATURE);
  }

  private static void configure(Supplier<REVLibError> config, SparkMax motor) {
    for (int i = 0; i < 5; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
    }
    DriverStation.reportError("Failure configuring spark max " + motor.getDeviceId() + "\n" + "Error: " + config.get().toString(), false);
  }

  public static void configureAndLog550(SparkMax motor, SparkMaxConfig config, boolean inverted, IdleMode idleMode) {
    configure(config, inverted, idleMode, 10, 40);
  }

  public static void configureCANStatusFrames(SparkMax motor, boolean velocityTemperatureVoltageCurrent, boolean position) {
    configureCANStatusFrames(motor, velocityTemperatureVoltageCurrent, position, false, false, false, false);
  }

  public static void configureCANStatusFrames(SparkMax motor, boolean velocityTemperatureVoltageCurrent, boolean position, boolean analogSensor, boolean alternateEncoder, boolean absoluteEncoderPosition, boolean absoluteEncoderVelocity) {

    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10), motor);
    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, velocityTemperatureVoltageCurrent ? 20 : 250), motor);
    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, position ? 250 : 250), motor);
    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, analogSensor ? 20 : 250), motor);
    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, alternateEncoder ? 20 : 250), motor);
    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, absoluteEncoderPosition ? 20 : 250), motor);
    // configure(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, absoluteEncoderVelocity ? 20 : 250), motor);
    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  }

  public static void configurePID(SparkMaxConfig config, double kP, double kI, double kD, double kV, boolean wrap) {
    config.closedLoop.pidf(kP, kI, kD, kV / 12.0);


    if (wrap) {
      config.closedLoop.positionWrappingEnabled(true);
      config.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);
    }

    // new TunableNumber(subsystem, "PID " + motor.getDeviceId(), pid::setP, 0.0);
  }

  public static void configureEncoder(SparkMaxConfig config, double encoderConversionFactor) {
    config.encoder.positionConversionFactor(encoderConversionFactor);
    config.encoder.velocityConversionFactor(encoderConversionFactor);
  }

  public static void configureFollower(SparkMaxConfig config, SparkMax leader, boolean inverted) {
    config.follow(leader, inverted);
  }
  
  public static void saveAndLog(Subsystem subsystem, SparkMax motor, SparkMaxConfig config) {
    configure(() -> motor.setCANTimeout(0), motor);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Logger.autoLog(subsystem, logPath + "current",          () -> motor.getOutputCurrent());
    // Logger.autoLog(subsystem, logPath + "voltage",          () -> motor.getBusVoltage());
    // Logger.autoLog(subsystem, logPath + "setOutput",        () -> motor.get());
    // Logger.autoLog(subsystem, logPath + "appliedOutput",    () -> motor.getAppliedOutput());
    // Logger.autoLog(subsystem, logPath + "appliedVoltage",   () -> motor.getBusVoltage() * motor.getAppliedOutput());
    // Logger.autoLog(subsystem, logPath + "powerWatts",       () -> motor.getBusVoltage() * motor.getAppliedOutput() * motor.getOutputCurrent());
    // Logger.autoLog(subsystem, logPath + "motorTemperature", () -> motor.getMotorTemperature());
    // Logger.autoLog(subsystem, logPath + "position",         () -> encoder.getPosition());
    // Logger.autoLog(subsystem, logPath + "velocity",         () -> encoder.getVelocity());
    
    StatusChecks.under(subsystem).add("Spark MAX", motor);
    // StatusChecks.addCheck(subsystem, logPath + "isTooHot", () -> motor.getMotorTemperature() <= NEO.SAFE_TEMPERATURE);
    // configure(() -> motor.burnFlash(), motor);
  }
}
