package frc.robot.util.hardware.motion;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.hardware.SparkMaxUtil;

/*
 * Uses oboard 1kHz PID, Feedforward, and Trapazoidal Profiles to
 * control a pivot mechanism precisely, smoothly, and accurately
 */

public class DualLinearActuator extends SubsystemBase {
  private Distance targetHeight = Meters.of(0.0);
  private double kS = 0.0;

  protected SparkMax leftMotor, rightMotor;
  private RelativeEncoder leftEncoder, rightEncoder;
  private SparkClosedLoopController leftPID, rightPID;

  private DutyCycleEncoder absoluteEncoder;
  private DigitalInput ceilingLimit, floorLimit;
  private Distance baseHeight, minHeight, maxHeight, tolerance;

  private double encoderOffset = 0.0;

  private Debouncer debouncer = new Debouncer(0.1);

  private Distance cycleHeight;

  private int cyclesCompleted;
  private Distance lastPosition = Meters.of(0.0);

  /**
   * Constructs a new DualLinearController.
   *
   * @param leftCAN The CAN ID for the left motor controller.
   * @param rightCAN The CAN ID for the right motor controller.
   * @param absoluteEncoderDIO The DIO port for the absolute encoder.
   * @param absolutePositionOffset The offset for the absolute encoder position.
   * @param kP The proportional gain for the PID controller.
   * @param kS The static gain for the PID controller.
   * @param sensorToMotorRatio The ratio of the sensor to motor.
   * @param mechanismToSensor The ratio of the mechanism to sensor.
   * @param baseHeight The base height of the mechanism.
   * @param minHeight The minimum height the mechanism can achieve.
   * @param maxHeight The maximum height the mechanism can achieve.
   * @param tolerance The tolerance for the height control.
   */
  public DualLinearActuator(
      String name,
      int leftCAN,
      int rightCAN,
      int absoluteEncoderDIO,
      int ceilingLimitDIO,
      int floorLimitDIO,
      double absolutePositionOffset,
      double kP,
      double kS,
      double gearing,
      Distance mechanismToSensor,
      Distance baseHeight,
      Distance minHeight,
      Distance maxHeight,
      Distance tolerance) {
    setName(name);

    this.kS = kS;
    this.baseHeight = baseHeight;
    this.minHeight = minHeight;
    this.maxHeight = maxHeight;
    this.tolerance = tolerance;
    this.cycleHeight = mechanismToSensor;
    encoderOffset = absolutePositionOffset;

    leftMotor = new SparkMax(leftCAN, MotorType.kBrushless);
    rightMotor = new SparkMax(rightCAN, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
    leftPID = leftMotor.getClosedLoopController();
    rightPID = rightMotor.getClosedLoopController();

    absoluteEncoder = new DutyCycleEncoder(absoluteEncoderDIO, 1.0, encoderOffset);
    lastPosition = getCycleDelta();

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    SparkMaxUtil.configure(motorConfig, true, IdleMode.kBrake);
    SparkMaxUtil.configureEncoder(motorConfig, cycleHeight.in(Meters) / gearing);
    SparkMaxUtil.configurePID(motorConfig, kP, 0.0, 0.0, 0.0, minHeight.in(Meters), maxHeight.in(Meters), false);
    SparkMaxUtil.saveAndLog(this, leftMotor, motorConfig);

    motorConfig = new SparkMaxConfig();
    SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
    SparkMaxUtil.configureEncoder(motorConfig, cycleHeight.in(Meters) / gearing);
    SparkMaxUtil.configurePID(motorConfig, kP, 0.0, 0.0, 0.0, minHeight.in(Meters), maxHeight.in(Meters), false);
    SparkMaxUtil.saveAndLog(this, rightMotor, motorConfig);

    this.ceilingLimit = new DigitalInput(ceilingLimitDIO);
    this.floorLimit = new DigitalInput(floorLimitDIO);
    // leftEncoder.setPosition(baseHeight.plus(getCycleDelta()).in(Meters));
    // rightEncoder.setPosition(baseHeight.plus(getCycleDelta()).in(Meters));
    leftEncoder.setPosition(baseHeight.in(Meters));
    rightEncoder.setPosition(baseHeight.in(Meters));

    Logger.logNumber(this.getName() + "/targetHeight", () -> getTargetHeight().in(Meters));
    Logger.logNumber(this.getName() + "/height", () -> getAverageHeight().in(Meters));
    // Logger.logNumber(this.getName() + "/leftHeight", () -> getLeftHeight().in(Meters));
    // Logger.logNumber(this.getName() + "/rightHeight", () -> getRightHeight().in(Meters));

    Logger.logBoolean(this.getName() + "/doneMoving", this::doneMoving);

    // Logger.logNumber(this.getName() + "/cycleDelta", () -> getCycleDelta().in(Meters));
    // Logger.logNumber(this.getName() + "/cyclesCompleted", () -> cyclesCompleted);
    // Logger.logNumber(this.getName() + "/cycledHeight", () -> calculateHeight().in(Meters));
    Logger.logNumber(this.getName() + "/rawAbsolutePosition", absoluteEncoder::get);

    Logger.logBoolean(this.getName() + "/ceilingLimit", this::triggeredCeilingLimit);
    Logger.logBoolean(this.getName() + "/floorLimit", this::triggeredFloorLimit);

    Logger.logMeasure(this.getName() + "/leftCurrent", () -> Amps.of(leftMotor.getOutputCurrent()));
    Logger.logMeasure(this.getName() + "/rightCurrent", () -> Amps.of(rightMotor.getOutputCurrent()));
    Logger.logBoolean(this.getName() + "/absoluteEncoderConnected", () -> absoluteEncoder.isConnected());

    // Logger.logNumber(this.getName() + "/offset", () -> encoderOffset);

    StatusChecks.Category statusChecks = StatusChecks.under(this);
    statusChecks.add("absoluteEncoderConnected", () -> absoluteEncoder.isConnected());
    statusChecks.add("absoluteEncoderUpdated", () -> absoluteEncoder.get() != 0.0);
    statusChecks.add("leftMotor", leftMotor);
    statusChecks.add("rightMotor", rightMotor);
  }

  private void seedEncoders(Distance height) {
    leftEncoder.setPosition(height.in(Meters));
    rightEncoder.setPosition(height.in(Meters));
  }

  public Distance getTargetHeight() {
    return targetHeight;
  }

  private Distance clampHeight(Distance height) {
    return Meters.of(MathUtil.clamp(height.in(Meters), minHeight.in(Meters), maxHeight.in(Meters)));
  }

  public Distance getLeftHeight() {
    return Meters.of(leftEncoder.getPosition());
  }

  public Distance getRightHeight() {
    return Meters.of(rightEncoder.getPosition());
  }

  public Distance getAverageHeight() {
    return getLeftHeight().plus(getRightHeight()).div(2);
  }

  public Distance getMaxHeight() {
    return maxHeight;
  }

  public Distance getMinHeight() {
    return minHeight;
  }

  public Distance getCycleDelta() {
    return cycleHeight.times(absoluteEncoder.get());
  }

  public Command move(double speed) {
    return runOnce(() -> moveSpeed(speed));
  }

  public void moveSpeed(double speed) {
    if (canMoveInDirection(speed)) {
      double startTime = Timer.getFPGATimestamp();
      leftMotor.set(speed);
      double swapTime = Timer.getFPGATimestamp();
      rightMotor.set(speed);
      double endTime = Timer.getFPGATimestamp();

      Logger.log(this.getName() + "/leftTime", swapTime - startTime);
      Logger.log(this.getName() + "/rightTime", endTime - swapTime);
    } else {
      stopMotors();
    }
  }

  public void moveUp() {
    moveSpeed(0.20);
  }

  public void moveDown() {
    moveSpeed(-0.20);
  }

  public boolean unsafeMoveDown() {
    if (!triggeredFloorLimit()) {
      leftMotor.set(-0.1);
      rightMotor.set(-0.1);
      return false;
    }

    stopMotors();
    return true;
  }

  public boolean doneMoving() {
    if (getTargetHeight() == null) return true;
    return debouncer.calculate(
        getAverageHeight().minus(targetHeight).abs(Meters) < tolerance.in(Meters));
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void moveTo(Distance requestedHeight) {
    targetHeight = clampHeight(requestedHeight);
    if (targetHeight == null) return; // If we havent set a target Height yet, do nothing

    if (!canMoveInDirection(requestedHeight.minus(getAverageHeight()).in(Meters))) {
      stopMotors();
      return;
    }

    leftPID.setReference(targetHeight.in(Meters), ControlType.kPosition, ClosedLoopSlot.kSlot0, kS);
    rightPID.setReference(
        targetHeight.in(Meters), ControlType.kPosition, ClosedLoopSlot.kSlot0, kS);
  }

  public boolean triggeredCeilingLimit() {
    return !ceilingLimit.get();
  }

  public boolean triggeredFloorLimit() {
    return !floorLimit.get();
  }

  public boolean canMoveInDirection(double velocity) {
    if (velocity > 0) {
      return /* getLeftHeight().lt(maxHeight) && getRightHeight().lt(maxHeight) && */ !triggeredCeilingLimit();
    } else {
      return /* getLeftHeight().gt(minHeight) && getRightHeight().gt(minHeight) && */ !triggeredFloorLimit();
    }
  }

  public Distance calculateHeight() {
    return cycleHeight.times(cyclesCompleted).plus(getCycleDelta());
  }

  @Override
  public void periodic() {
    if (lastPosition.gt(cycleHeight.minus(tolerance)) && getCycleDelta().lt(tolerance)) {
      cyclesCompleted++;
    }

    if (lastPosition.lt(tolerance) && getCycleDelta().lt(cycleHeight.minus(tolerance))) {
      cyclesCompleted--;
    }

    if (triggeredFloorLimit()) {
      seedEncoders(baseHeight);
    }

    lastPosition = getCycleDelta();
  }
}
