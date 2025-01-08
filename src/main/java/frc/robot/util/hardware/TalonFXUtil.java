package frc.robot.util.hardware;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.NEO;
import frc.robot.util.software.Logging.StatusChecks;

public final class TalonFXUtil {
  public static void configure(TalonFXConfiguration config, InvertedValue inverted, NeutralModeValue neutralMode) {
    configure(config, inverted, neutralMode, NEO.SAFE_FREE_CURRENT, NEO.SAFE_STALL_CURRENT);
  }

  public static void configure(TalonFXConfiguration config, InvertedValue inverted, NeutralModeValue neutralMode, int supplyCurrentLimit, int statorCurrentLimit) {
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;

    config.MotorOutput.NeutralMode = neutralMode;
    config.MotorOutput.Inverted = inverted;

    // config.voltageCompensation(12.0);
    // config.openLoopRampRate(NEO.SAFE_RAMP_RATE);
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

  public static void configureEncoder(SparkMaxConfig config, double encoderConversionFactor) {
    config.encoder.positionConversionFactor(encoderConversionFactor);
    config.encoder.velocityConversionFactor(encoderConversionFactor);
  }

  public static void saveAndLog(Subsystem subsystem, TalonFX motor, TalonFXConfiguration config) {
    motor.getConfigurator().apply(config);
    String logPath = "motor" + motor.getDeviceID() + "/";

    // Logger.autoLog(subsystem, logPath + "current",          () -> motor.getOutputCurrent());
    // Logger.autoLog(subsystem, logPath + "voltage",          () -> motor.getBusVoltage());
    // Logger.autoLog(subsystem, logPath + "setOutput",        () -> motor.get());
    // Logger.autoLog(subsystem, logPath + "appliedOutput",    () -> motor.getAppliedOutput());
    // Logger.autoLog(subsystem, logPath + "appliedVoltage",   () -> motor.getBusVoltage() * motor.getAppliedOutput());
    // Logger.autoLog(subsystem, logPath + "powerWatts",       () -> motor.getBusVoltage() * motor.getAppliedOutput() * motor.getOutputCurrent());
    // Logger.autoLog(subsystem, logPath + "motorTemperature", () -> motor.getMotorTemperature());
    // Logger.autoLog(subsystem, logPath + "position",         () -> encoder.getPosition());
    // Logger.autoLog(subsystem, logPath + "velocity",         () -> encoder.getVelocity());
    
    StatusChecks.addCheck(subsystem, logPath + "hasFaults", () -> motor.getFaultField().getValue() == 0);
    StatusChecks.addCheck(subsystem, logPath + "isConnected", () -> motor.isConnected());
    // StatusChecks.addCheck(subsystem, logPath + "isTooHot", () -> motor.getMotorTemperature() <= NEO.SAFE_TEMPERATURE);
    // configure(() -> motor.burnFlash(), motor);
  }
}
