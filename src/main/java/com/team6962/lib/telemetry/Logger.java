package com.team6962.lib.telemetry;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.studica.frc.AHRS;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public class Logger {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("Logs");
  private static List<Runnable> updates = new ArrayList<>();
  private static Notifier notifier = new Notifier(Logger::update);
  private static Field2d field2d = new Field2d();

  public static void start(Time period) {
    System.out.println("Starting periodic");
    notifier.startPeriodic(period.in(Seconds));
    SmartDashboard.putData(field2d);
  }

  private static void update() {
    System.out.println("Updating " + updates.size() + " fields");
    updates.forEach(Runnable::run);
  }

  public static Field2d getField() {
    return field2d;
  }

  public static void logBoolean(String key, Supplier<Boolean> supplier) {
    updates.add(() -> log(key, supplier.get()));
  }

  public static void log(String key, Boolean value) {
    table.getEntry(key).setBoolean(value);
  }

  public static void logString(String key, Supplier<String> supplier) {
    updates.add(() -> log(key, supplier.get()));
  }

  public static void log(String key, String value) {
    table.getEntry(key).setString(value);
  }

  public static void logNumber(String key, Supplier<Number> supplier) {
    updates.add(() -> log(key, supplier.get()));
  }

  public static void log(String key, Number value) {
    table.getEntry(key).setNumber(value);
  }

  public static <T extends Unit> void logMeasure(String key, Supplier<Measure<T>> supplier) {
    updates.add(() -> log(key, supplier.get()));
  }

  public static <T extends Unit> void log(String key, Measure<T> value) {
    table.getEntry(key + reformatMeasureName(value.unit().name())).setValue(value.in(value.unit()));
  }

  private static String reformatMeasureName(String name) {
    return Arrays.stream(name.split(" "))
        .map(str -> str.substring(0, 1).toUpperCase() + str.substring(1).toLowerCase())
        .reduce("", (a, b) -> a + b);
  }

  public static void logBooleanArray(String key, Supplier<boolean[]> supplier) {
    updates.add(() -> log(key, supplier.get()));
  }

  public static void log(String key, boolean[] value) {
    updates.add(() -> table.getEntry(key).setBooleanArray(value));
  }

  public static void logLongArray(String key, Supplier<long[]> supplier) {
    updates.add(() -> log(key, supplier.get()));
  }

  public static void log(String key, long[] value) {
    updates.add(() -> table.getEntry(key).setIntegerArray(value));
  }

  public static void logDoubleArray(String key, Supplier<double[]> supplier) {
    updates.add(() -> log(key, supplier.get()));
  }

  public static void log(String key, double[] value) {
    updates.add(() -> table.getEntry(key).setDoubleArray(value));
  }

  public static void logNumberArray(String key, Supplier<Number[]> supplier) {
    updates.add(() -> log(key, supplier.get()));
  }

  public static void log(String key, Number[] supplier) {
    updates.add(() -> table.getEntry(key).setValue(supplier));
  }

  public static void logTranslation(String key, Supplier<Translation2d> supplier) {
    updates.add(() -> log(key, supplier.get()));
  }

  public static void log(String key, Translation2d value) {
    updates.add(
        () -> table.getEntry(key).setDoubleArray(new double[] {value.getX(), value.getY()}));
  }

  public static void logRotation(String key, Supplier<Rotation2d> supplier) {
    updates.add(() -> log(key, supplier.get()));
  }

  public static void log(String key, Rotation2d value) {
    table.getEntry(key + "Radians").setDouble(value.getRadians());
  }

  public static void logSpeeds(String key, Supplier<ChassisSpeeds> supplier) {
    updates.add(() -> log(key, supplier.get()));
  }

  public static void log(String key, ChassisSpeeds speeds) {
    table
        .getEntry(key)
        .setDoubleArray(
            new double[] {
              speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond
            });
  }

  public static void logPose(String key, Supplier<Pose2d> supplier) {
    updates.add(() -> log(key, supplier.get()));
  }

  public static void log(String key, Pose2d pose) {
    table
        .getEntry(key)
        .setDoubleArray(new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()});
  }

  public static void logSwerveModuleState(String path, Supplier<SwerveModuleState> supplier) {
    updates.add(() -> log(path, supplier.get()));
  }

  public static void log(String path, SwerveModuleState state) {
    log(path + "/speedMetersPerSecond", state.speedMetersPerSecond);
    log(path + "/angleRadians", state.angle.getRadians());
  }

  public static void logSwerveModuleStates(String path, Supplier<SwerveModuleState[]> supplier) {
    updates.add(() -> log(path, supplier.get()));
  }

  public static void log(String path, SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      log(path + "/" + i, states[i]);
    }
  }

  public static void logSwerveModulePosition(String path, Supplier<SwerveModulePosition> supplier) {
    updates.add(() -> log(path, supplier.get()));
  }

  public static void log(String path, SwerveModulePosition position) {
    log(path + "/distanceMeters", position.distanceMeters);
    log(path + "/angleRadians", position.angle.getRadians());
  }

  public static void logSwerveModulePositions(
      String path, Supplier<SwerveModulePosition[]> supplier) {
    updates.add(() -> log(path, supplier.get()));
  }

  public static void log(String path, SwerveModulePosition[] positions) {
    for (int i = 0; i < positions.length; i++) {
      log(path + "/" + i, positions[i]);
    }
  }

  public static void logNavX(String path, Supplier<AHRS> supplier) {
    updates.add(() -> log(path, supplier.get()));
  }

  public static void log(String path, AHRS navX) {
    log(path + "/isAltitudeValid", navX.isAltitudeValid());
    log(path + "/isCalibrating", navX.isCalibrating());
    log(path + "/isConnected", navX.isConnected());
    log(path + "/isMagneticDisturbance", navX.isMagneticDisturbance());
    log(path + "/isMagnetometerCalibrated", navX.isMagnetometerCalibrated());
    log(path + "/isMoving", navX.isMoving());
    log(path + "/isRotating", navX.isRotating());
    log(path + "/actualUpdateRate", navX.getActualUpdateRate());
    log(path + "/firmwareVersion", navX.getFirmwareVersion());
    log(path + "/altitude", navX.getAltitude());
    log(path + "/angle", navX.getAngle());
    log(path + "/angleAdjustment", navX.getAngleAdjustment());
    log(path + "/compassHeading", navX.getCompassHeading());
    log(path + "/displacementX", navX.getDisplacementX());
    log(path + "/displacementY", navX.getDisplacementY());
    log(path + "/displacementZ", navX.getDisplacementZ());
    log(path + "/fusedHeading", navX.getFusedHeading());
    log(path + "/pitch", navX.getPitch());
    log(path + "/pressure", navX.getPressure());
    log(path + "/roll", navX.getRoll());
    log(path + "/yaw", navX.getYaw());
    log(path + "/temperature", navX.getTempC());
    log(path + "/velocityX", navX.getVelocityX());
    log(path + "/velocityY", navX.getVelocityY());
    log(path + "/velocityZ", navX.getVelocityZ());
    log(path + "/accelerationX", navX.getRawAccelX());
    log(path + "/accelerationY", navX.getRawAccelY());
    log(path + "/accelerationZ", navX.getRawAccelZ());
  }

  public static void logStatusSignal(String path, Supplier<StatusSignal<?>> supplier) {
    updates.add(() -> log(path, supplier.get()));
  }

  public static void log(String path, StatusSignal<?> supplier) {
    log(path + "/status", supplier.getStatus().toString());
    logObject(path + "/value", supplier.getValue());
  }

  public static void logCANcoder(String path, Supplier<CANcoder> supplier) {
    updates.add(() -> log(path, supplier.get()));
  }

  public static void log(String path, CANcoder encoder) {
    log(path + "/absolutePosition", encoder.getAbsolutePosition());
    log(path + "/position", encoder.getPosition());
    log(path + "/velocity", encoder.getVelocity());
  }

  public static void logCANStatus(String path, Supplier<CANStatus> supplier) {
    updates.add(() -> log(path, supplier.get()));
  }

  public static void log(String path, CANStatus canStatus) {
    log(path + "/percentBusUtilization", canStatus.percentBusUtilization);
    log(path + "/busOffCount", canStatus.busOffCount);
    log(path + "/txFullCount", canStatus.txFullCount);
    log(path + "/receiveErrorCount", canStatus.receiveErrorCount);
    log(path + "/transmitErrorCount", canStatus.transmitErrorCount);
  }

  public static void logRoboRIO(String path) {
    updates.add(() -> logRoboRIOOnce(path));
  }

  public static void logRoboRIOOnce(String path) {
    log(path + "/isBrownedOut", RobotController.isBrownedOut());
    log(path + "/isSysActive", RobotController.isSysActive());
    log(path + "/brownoutVoltage", RobotController.getBrownoutVoltage());
    log(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    log(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    log(path + "/inputCurrent", RobotController.getInputCurrent());
    log(path + "/inputVoltage", RobotController.getInputVoltage());
    log(path + "/3V3Line/current", RobotController.getCurrent3V3());
    log(path + "/5VLine/current", RobotController.getCurrent5V());
    log(path + "/6VLine/current", RobotController.getCurrent6V());
    log(path + "/3V3Line/enabled", RobotController.getEnabled3V3());
    log(path + "/5VLine/enabled", RobotController.getEnabled5V());
    log(path + "/6VLine/enabled", RobotController.getEnabled6V());
    log(path + "/3V3Line/faultCount", RobotController.getFaultCount3V3());
    log(path + "/5VLine/faultCount", RobotController.getFaultCount5V());
    log(path + "/6VLine/faultCount", RobotController.getFaultCount6V());
    log(path + "/3V3Line/voltage", RobotController.getVoltage3V3());
    log(path + "/5VLine/voltage", RobotController.getVoltage5V());
    log(path + "/6VLine/voltage", RobotController.getVoltage6V());
    log(path + "/canStatus", RobotController.getCANStatus());
  }

  public static void logPowerDistribution(String path, Supplier<PowerDistribution> supplier) {
    updates.add(() -> log(path, supplier.get()));
  }

  public static void log(String path, PowerDistribution PDH) {
    log(path + "/faults", PDH.getFaults());
    log(path + "/canId", PDH.getModule());
    for (int i = 0; i <= 23; i++) {
      log(path + "/channels/channel" + i + "Current", PDH.getCurrent(i));
    }
    log(path + "/isSwitchableChannelOn", PDH.getSwitchableChannel());
    log(path + "/temperature", PDH.getTemperature());
    log(path + "/totalCurrent", PDH.getTotalCurrent());
    log(path + "/totalJoules", PDH.getTotalEnergy());
    log(path + "/totalWatts", PDH.getTotalPower());
    log(path + "/voltage", PDH.getVoltage());
  }

  public static void logPowerDistributionFaults(
      String path, Supplier<PowerDistributionFaults> supplier) {
    updates.add(() -> log(path, supplier.get()));
  }

  public static void log(String path, PowerDistributionFaults faults) {
    log(path + "/brownout", faults.Brownout);
    log(path + "/canWarning", faults.CanWarning);

    for (int i = 0; i < 24; i++) {
      log(path + "/channel" + i + "BreakerFault", faults.getBreakerFault(i));
    }

    log(path + "/hardwareFault", faults.HardwareFault);
  }

  private static void logObjectRecurring(String path, Supplier<?> supplier) {
    updates.add(() -> logObject(path, supplier.get()));
  }

  /**
   * Safely logs an unknown type of object to NetworkTables. If the object is a known type, it will
   * be logged using one of the other log methods. If the object is some other type, it will attempt
   * to log each of its fields recursively. No exceptions will be thrown, but access exceptions will
   * be logged in place of the values of inaccessible fields.
   *
   * @param path The path to log the object to
   * @param object The object to log
   */
  public static void logObject(String path, Object object) {
    if (object instanceof Boolean) log(path, (Boolean) object);
    else if (object instanceof String) log(path, (String) object);
    else if (object instanceof Number) log(path, (Number) object);
    else if (object instanceof Measure) log(path, (Measure<?>) object);
    else if (object instanceof boolean[]) log(path, (boolean[]) object);
    else if (object instanceof long[]) log(path, (long[]) object);
    else if (object instanceof double[]) log(path, (double[]) object);
    else if (object instanceof Number[]) log(path, (Number[]) object);
    else if (object instanceof Translation2d) log(path, (Translation2d) object);
    else if (object instanceof Rotation2d) log(path, (Rotation2d) object);
    else if (object instanceof ChassisSpeeds) log(path, (ChassisSpeeds) object);
    else if (object instanceof Pose2d) log(path, (Pose2d) object);
    else if (object instanceof SwerveModuleState) log(path, (SwerveModuleState) object);
    else if (object instanceof SwerveModuleState[]) log(path, (SwerveModuleState[]) object);
    else if (object instanceof SwerveModulePosition) log(path, (SwerveModulePosition) object);
    else if (object instanceof AHRS) log(path, (AHRS) object);
    else if (object instanceof StatusSignal<?>) log(path, (StatusSignal<?>) object);
    else if (object instanceof CANcoder) log(path, (CANcoder) object);
    else if (object instanceof CANStatus) log(path, (CANStatus) object);
    else if (object instanceof PowerDistribution) log(path, (PowerDistribution) object);
    else if (object instanceof PowerDistributionFaults) log(path, (PowerDistributionFaults) object);
    else if (object instanceof Supplier) logObjectRecurring(path, (Supplier<?>) object);
    else if (object instanceof Object[]) {
      for (int i = 0; i < ((Object[]) object).length; i++) {
        logObject(path + "/" + i, ((Object[]) object)[i]);
      }
    } else if (object instanceof Collection<?>) {
      int i = 0;
      for (var element : (Collection<?>) object) {
        logObject(path + "/" + i, element);
        i++;
      }
    } else if (object instanceof Map<?, ?>) {
      for (var entry : ((Map<?, ?>) object).entrySet()) {
        logObject(path + "/" + entry.getKey().toString(), entry.getValue());
      }
    } else {
      try {
        table.getEntry(path).setValue(object);
      } catch (IllegalArgumentException valueException) {
        try {
          for (var field : object.getClass().getFields()) {
            String subpath = path + "/" + field.getName();

            try {
              logObject(path, field.get(object));
            } catch (IllegalAccessException accessException) {
              log(subpath, accessException.getMessage());
            }
          }
        } catch (SecurityException securityException) {
          log(path, securityException.getMessage());
        }
      }
    }
  }
}
