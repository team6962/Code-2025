package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.hardware.SparkMaxUtil;
import java.util.function.BooleanSupplier;

public class ManipulatorGrabber extends SubsystemBase {
  private SparkMax motor;
  private double intakeSpeed;
  private double dropSpeed;
  private BooleanSupplier isEnabled;

  public final ManipulatorSensor sensor;

  public ManipulatorGrabber(
      int motorId,
      ManipulatorSensor sensor,
      double intakeSpeed,
      double dropSpeed,
      BooleanSupplier isEnabled) {
    motor = new SparkMax(motorId, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxUtil.configure(config, false, IdleMode.kCoast);
    SparkMaxUtil.saveAndLog(this, motor, config);

    this.sensor = sensor;
    this.intakeSpeed = intakeSpeed;
    this.dropSpeed = dropSpeed;
  }

  public Command run(double speed) {
    return Commands.run(() -> motor.set(isEnabled.getAsBoolean() ? speed : 0), this);
  }

  public Command intake() {
    return run(intakeSpeed).until(sensor::hasGamePiece).alongWith(sensor.duringIntake());
  }

  public Command drop() {
    return run(dropSpeed).until(() -> !sensor.hasGamePiece()).alongWith(sensor.duringDrop());
  }

  public Command stop() {
    return Commands.run(motor::disable, this);
  }

  public static interface ManipulatorSensor {
    public boolean hasGamePiece();

    public default Command duringIntake() {
      return Commands.none();
    }

    public default Command duringDrop() {
      return Commands.none();
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
  }

  public static class CurrentSensor implements ManipulatorSensor {
    private boolean hasGamePiece;
    private Debouncer stallDebouncer;
    private SparkMax motor;

    public CurrentSensor(boolean startsWithGamePiece, int motorID) {
      this.motor = new SparkMax(motorID, MotorType.kBrushless);
      this.hasGamePiece = startsWithGamePiece;
      this.stallDebouncer = new Debouncer(0.4, DebounceType.kRising);
    }

    @Override
    public boolean hasGamePiece() {
      return hasGamePiece;
    }

    private boolean isStalled() {
      return stallDebouncer.calculate(motor.getEncoder().getVelocity() == 0.0 && motor.getOutputCurrent() > 0.0);
    }

    @Override
    public Command duringIntake() {
      return Commands.waitUntil(this::isStalled).andThen(() -> hasGamePiece = true);
    }

    @Override
    public Command duringDrop() {
      return Commands.runOnce(() -> hasGamePiece = false);
    }
    
  }
}
