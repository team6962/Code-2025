package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.team6962.lib.telemetry.Logger;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.hardware.SparkMaxUtil;

public class ManipulatorGrabber extends SubsystemBase {
  private Motor[] motors;
  private double intakeSpeed;
  private double dropSpeed;
  private BooleanSupplier isEnabled;

  public final ManipulatorSensor sensor;

  public ManipulatorGrabber(
      MotorConfig[] motors,
      ManipulatorSensor sensor,
      double intakeSpeed,
      double dropSpeed,
      BooleanSupplier isEnabled) {
    this.motors = new Motor[motors.length];

    for (int i = 0; i < motors.length; i++) {
      this.motors[i] = new Motor(motors[i]);
    }

    this.sensor = sensor;
    this.intakeSpeed = intakeSpeed;
    this.dropSpeed = dropSpeed;

    sensor.addManipulatorProperties(this.motors);

    setDefaultCommand(hold());
  }

  public record MotorConfig(int motorId, boolean inverted) {
  }

  private class Motor {
    private SparkMax motor;

    public Motor(MotorConfig config) {
      this(config.motorId, config.inverted);
    }

    public Motor(int motorId, boolean inverted) {
      motor = new SparkMax(motorId, MotorType.kBrushless);

      SparkMaxConfig config = new SparkMaxConfig();
      SparkMaxUtil.configureAndLog550(motor, config, inverted, IdleMode.kBrake);
      
      Logger.log(getName() + "/Motor " + motorId + "/Current", Amps.of(motor.getOutputCurrent()));
    }

    public SparkMax getSparkMax() {
      return motor;
    }

    public void set(double speed) {
      motor.set(speed);
    }
  }

  public Command hold() {
    return Commands.run(() -> {
      double limitedSpeed = isEnabled.getAsBoolean() ? (sensor.needsHold() ? intakeSpeed : 0) : 0;

      for (Motor motor : motors) {
        motor.set(limitedSpeed);
      }
    }, this);
  }

  public Command run(double speed) {
    return Commands.run(() -> {
      double limitedSpeed = isEnabled.getAsBoolean() ? speed : 0;

      for (Motor motor : motors) {
        motor.set(limitedSpeed);
      }
    }, this);
  }

  public Command intake() {
    return run(intakeSpeed).until(sensor::hasGamePiece).alongWith(sensor.duringIntake());
  }

  public Command drop() {
    return run(dropSpeed).until(() -> !sensor.hasGamePiece()).alongWith(sensor.duringDrop());
  }

  public Command stop() {
    return Commands.run(() -> {
      for (Motor motor : motors) {
        motor.set(0);
      }
    }, this);
  }

  public static interface ManipulatorSensor {
    public boolean hasGamePiece();

    public default Command duringIntake() {
      return Commands.none();
    }

    public default Command duringDrop() {
      return Commands.none();
    }

    public default void addManipulatorProperties(Motor[] motors) {
    }

    public default boolean needsHold() {
      return false;
    }
  }

  public static class DigitalSensor implements ManipulatorSensor {
    private DigitalInput sensor;

    public DigitalSensor(DigitalInput sensor) {
      this.sensor = sensor;
    }

    public DigitalSensor(int channel) {
      this(new DigitalInput(channel));
    }

    @Override
    public boolean hasGamePiece() {
      return !sensor.get();
    }

    @Override
    public boolean needsHold() {
        return false;
    }
  }

  public static class TimeSensor implements ManipulatorSensor {
    private boolean hasGamePiece;
    private Time intakeTime;
    private Time dropTime;

    public TimeSensor(boolean startsWithGamePiece, Time intakeTime, Time dropTime) {
      this.hasGamePiece = startsWithGamePiece;
      this.intakeTime = intakeTime;
      this.dropTime = dropTime;
    }

    @Override
    public boolean hasGamePiece() {
      return hasGamePiece;
    }

    @Override
    public Command duringIntake() {
      return Commands.waitTime(intakeTime).andThen(() -> hasGamePiece = true);
    }

    @Override
    public Command duringDrop() {
      return Commands.waitTime(dropTime).andThen(() -> hasGamePiece = false);
    }

    @Override
    public boolean needsHold() {
        return false;
    }
  }

  public static class CurrentSensor extends SubsystemBase implements ManipulatorSensor {
    private boolean hasGamePiece;
    private Debouncer stallDebouncer;
    private Motor[] motors;
    private boolean stalled;
    private Current threshold;

    public CurrentSensor(boolean startsWithGamePiece, Current threshold) {
      this.hasGamePiece = startsWithGamePiece;
      this.stallDebouncer = new Debouncer(0.4, DebounceType.kBoth);
      this.threshold = threshold;
    }
    
    @Override
    public void addManipulatorProperties(Motor[] motors) {
        this.motors = motors;
    }

    @Override
    public boolean hasGamePiece() {
      return hasGamePiece;
    }

    private boolean isStalled() {
      return stalled;
    }

    @Override
    public Command duringIntake() {
      return Commands.waitUntil(this::isStalled).andThen(() -> hasGamePiece = true);
    }

    @Override
    public Command duringDrop() {
      return Commands.waitUntil(() -> !isStalled()).andThen(() -> hasGamePiece = false);
    }

    @Override
    public void periodic() {
      MutCurrent totalCurrent = Amps.mutable(0);

      for (Motor motor : motors) {
        totalCurrent = totalCurrent.mut_plus(Amps.of(motor.getSparkMax().getOutputCurrent()));
      }

      Current averageCurrent = totalCurrent.div(motors.length);

      stalled = stallDebouncer.calculate(averageCurrent.gt(threshold));
    }

    @Override
    public boolean needsHold() {
        return !isStalled();
    }
  }
}
