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
  private DigitalInput ceilingLimit, floorLimit;
  private Distance baseHeight, minHeight, maxHeight, tolerance;

  private Debouncer debouncer = new Debouncer(0.1);

  private Distance cycleHeight;

  private boolean zeroed = false;

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
      int ceilingLimitDIO,
      int floorLimitDIO,
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

    leftMotor = new SparkMax(leftCAN, MotorType.kBrushless);
    rightMotor = new SparkMax(rightCAN, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
    leftPID = leftMotor.getClosedLoopController();
    rightPID = rightMotor.getClosedLoopController();

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    SparkMaxUtil.configure(motorConfig, true, IdleMode.kBrake);
    SparkMaxUtil.configureEncoder(motorConfig, cycleHeight.in(Meters) / gearing);
    SparkMaxUtil.configurePID(
        motorConfig, kP, 0.0, 0.0, 0.0, minHeight.in(Meters), maxHeight.in(Meters), false);
    SparkMaxUtil.saveAndLog(this, leftMotor, motorConfig);

    motorConfig = new SparkMaxConfig();
    SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
    SparkMaxUtil.configureEncoder(motorConfig, cycleHeight.in(Meters) / gearing);
    SparkMaxUtil.configurePID(
        motorConfig, kP, 0.0, 0.0, 0.0, minHeight.in(Meters), maxHeight.in(Meters), false);
    SparkMaxUtil.saveAndLog(this, rightMotor, motorConfig);

    this.ceilingLimit = new DigitalInput(ceilingLimitDIO);
    this.floorLimit = new DigitalInput(floorLimitDIO);
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

    Logger.logBoolean(this.getName() + "/limits/ceil", this::triggeredCeilingLimit);
    Logger.logBoolean(this.getName() + "/limits/floor", this::triggeredFloorLimit);

    Logger.logMeasure(
        this.getName() + "/motors/left/current", () -> Amps.of(leftMotor.getOutputCurrent()));
    Logger.logNumber(this.getName() + "/motors/left/dutycycle", () -> leftMotor.getAppliedOutput());

    Logger.logMeasure(
        this.getName() + "/motors/right/current", () -> Amps.of(rightMotor.getOutputCurrent()));
    Logger.logNumber(
        this.getName() + "/motors/right/dutycycle", () -> rightMotor.getAppliedOutput());

    // Logger.logNumber(this.getName() + "/offset", () -> encoderOffset);

    StatusChecks.Category statusChecks = StatusChecks.under(this);
    statusChecks.add("leftMotor", leftMotor);
    statusChecks.add("rightMotor", rightMotor);
    seedEncoders(maxHeight);
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

  public Command move(double speed) {
    return runEnd(() -> moveSpeed(speed), this::stopMotors);
  }

  public void moveSpeed(double speed) {
    if (canMoveInDirection(speed)) {
      leftMotor.set(speed);
      rightMotor.set(speed);
    } else {
      stopMotors();
    }
  }

  public Command up() {
    return move(0.2);
  }

  public Command down() {
    return move(-0.2);
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

  public boolean inRange(Distance height) {
    if (height == null) return true;
    return debouncer.calculate(getAverageHeight().minus(height).abs(Meters) < tolerance.in(Meters));
  }

  public boolean doneMoving() {
    return inRange(targetHeight);
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
    if (!zeroed) {
      return true;
    }
    return !ceilingLimit.get();
  }

  public boolean triggeredFloorLimit() {
    if (!floorLimit.get()) {
      zeroed = true;
      return true;
    }
    return false;
  }

  public boolean canMoveInDirection(double velocity) {
    if (velocity > 0) {
      return /* getLeftHeight().lt(maxHeight) && getRightHeight().lt(maxHeight) && */ !triggeredCeilingLimit();
    } else {
      return /* getLeftHeight().gt(minHeight) && getRightHeight().gt(minHeight) && */ !triggeredFloorLimit();
    }
  }

  @Override
  public void periodic() {
    if (triggeredFloorLimit()) {
      seedEncoders(baseHeight);
    }
  }
}
