package com.team6962.lib.telemetry;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * {@code StatusChecks} is a utility class that provides a way to add boolean status checks to
 * Shuffleboard. It is intended to be used to monitor the health of various subsystems and
 * components before and during a match.
 *
 * <p>You can add a check by calling the {@code add} method on a {@code Category} object, gotten by
 * calling the {@code under} method on a {@code StatusChecks} instance:
 *
 * <pre>
 * statusChecks.under("Swerve Drive").add("Is Working", () -> swerveDrive.isWorking());
 * </pre>
 *
 * <p>You can also add checks for TalonFX controllers, CANcoders, and Spark MAX controllers:
 *
 * <pre>
 * statusChecks.under("Shooter").add("Pivot Motor", pivotMotor);
 * </pre>
 */
public final class StatusChecks {
  private StatusChecks() {}

  private static ShuffleboardTab tab = Shuffleboard.getTab("Status Checks");
  private static Notifier notifier = new Notifier(StatusChecks::refresh);
  private static final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("StatusChecks");
  private static int currentPosition = 0;
  private static int viewWidth = 10;

  private static int getColumn(int position) {
    return position % viewWidth;
  }

  private static int getRow(int position) {
    return position / viewWidth;
  }

  public static void start() {
    tab.add("Refresh", Commands.runOnce(StatusChecks::refresh));
  }

  public static void start(Time autoRefreshPeriod) {
    start();

    notifier.startPeriodic(autoRefreshPeriod.in(Seconds));
  }

  private static List<Runnable> updates = new ArrayList<>();

  public static void refresh() {
    updates.forEach(Runnable::run);
  }

  public static Category under(String name) {
    return new Category(name);
  }

  public static Category under(Subsystem subsystem) {
    return under(subsystem.getName());
  }

  public static class Category {
    private final NetworkTable table;
    private String categoryName;

    public Category(String name) {
      table = networkTable.getSubTable(name);
      this.categoryName = name;
    }

    public void add(String name, BooleanSupplier checkSupplier) {
      int position = currentPosition++;

      GenericEntry entry =
          tab.add(categoryName + "/" + name, checkSupplier.getAsBoolean())
              .withWidget(BuiltInWidgets.kBooleanBox)
              .withSize(1, 1)
              .withPosition(getColumn(position), getRow(position))
              .getEntry();
      
      NetworkTableEntry ntEntry = table.getEntry(name);
      ntEntry.setBoolean(checkSupplier.getAsBoolean());

      updates.add(() -> entry.setBoolean(checkSupplier.getAsBoolean()));
      updates.add(
          () -> {
            if (!checkSupplier.getAsBoolean())
              DriverStation.reportError("===== STATUS CHECKS FAILED ====", false);
          });
    }

    public void add(String name, TalonFX motor) {
      add(motor.getDeviceID() + ". " + name + " Alive", () -> motor.isAlive());
      add(
          motor.getDeviceID() + ". " + name + " Faults",
          () ->
              CTREUtils.unwrap(motor.getFaultField()) != 0
                  || CTREUtils.unwrap(motor.getStickyFaultField()) != 0);
    }

    public void add(String name, CANcoder encoder) {
      add(
          encoder.getDeviceID() + ". " + name + " Alive",
          () -> encoder.getVersion().getStatus().isOK());
      add(
          encoder.getDeviceID() + ". " + name + " Faults",
          () ->
              CTREUtils.unwrap(encoder.getFaultField()) != 0
                  || CTREUtils.unwrap(encoder.getStickyFaultField()) != 0);
    }

    public void add(String name, SparkMax motor) {
      add(motor.getDeviceId() + ". " + name + " Alive", () -> motor.getFirmwareVersion() != 0);
      add(
          motor.getDeviceId() + ". " + name + " Faults",
          () -> motor.getFaults().rawBits != 0 || motor.getStickyFaults().rawBits != 0);
    }

    public void add(String name, DutyCycleEncoder encoder) {
      add(name + " Connected", () -> encoder.isConnected());
    }
  }
}
