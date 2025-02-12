package frc.robot.util.hardware.linear;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.hardware.SparkMaxUtil;

/*
 * Uses oboard 1kHz PID, Feedforward, and Trapazoidal Profiles to
 * control a pivot mechanism precisely, smoothly, and accurately
 */

public class LinearController extends SubsystemBase {
  private Distance targetHeight = Meters.of(0.0);
  private double kS = 0.0;

  private SparkMax leftMotor, rightMotor;
  private RelativeEncoder leftEncoder, rightEncoder;
  private SparkClosedLoopController leftPID, rightPID;

  private DutyCycleEncoder absoluteEncoder;

  private Distance minHeight, maxHeight, tolerance;

  private Debouncer debouncer = new Debouncer(0.1);

  private LinearRatios ratios;
  private DCMotor motors;
  private Mass carriageMass;

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
   * @param minHeight The minimum height the mechanism can achieve.
   * @param maxHeight The maximum height the mechanism can achieve.
   * @param tolerance The tolerance for the height control.
   */
  public LinearController(
      int leftCAN,
      int rightCAN,
      int absoluteEncoderDIO,
      Angle absolutePositionOffset,
      double kP,
      double kS,
      LinearRatios ratios,
      Distance minHeight,
      Distance maxHeight,
      Distance tolerance,
      DCMotor motors,
      Mass carriageMass) {
    this.ratios = ratios;
    this.motors = motors;
    this.carriageMass = carriageMass;

    this.kS = kS;
    this.minHeight = minHeight;
    this.maxHeight = maxHeight;
    this.tolerance = tolerance;

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    leftMotor = new SparkMax(leftCAN, MotorType.kBrushless);
    rightMotor = new SparkMax(rightCAN, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
    leftPID = leftMotor.getClosedLoopController();
    rightPID = rightMotor.getClosedLoopController();

    absoluteEncoder = new DutyCycleEncoder(
      absoluteEncoderDIO, 1.0,
      ratios.sensorToCarriage(absolutePositionOffset).in(Meters)
    );

    Logger.log(this.getName() + "/leftB4Config", leftEncoder.getPosition());
    Logger.log(this.getName() + "/rightB4Config", rightEncoder.getPosition());
    SparkMaxUtil.configure(motorConfig, true, IdleMode.kBrake);
    SparkMaxUtil.configureEncoder(motorConfig, ratios.sensorRotationsToCarriageMeters());
    SparkMaxUtil.configurePID(motorConfig, kP, 0.0, 0.0, 0.0, false);
    SparkMaxUtil.saveAndLog(this, leftMotor, motorConfig);

    motorConfig = new SparkMaxConfig();
    SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
    SparkMaxUtil.configureEncoder(motorConfig, ratios.rotorRotationsToCarriageMeters());
    SparkMaxUtil.configurePID(motorConfig, kP, 0.0, 0.0, 0.0, false);
    SparkMaxUtil.saveAndLog(this, rightMotor, motorConfig);

    leftEncoder.setPosition(getCycleDelta().in(Meters));
    rightEncoder.setPosition(getCycleDelta().in(Meters));

    Logger.log(this.getName() + "/leftAfterConfig", leftEncoder.getPosition());
    Logger.log(this.getName() + "/rightAfterConfig", rightEncoder.getPosition());

    Logger.logNumber(this.getName() + "/targetHeight", () -> getTargetHeight().in(Meters));
    Logger.logNumber(this.getName() + "/height", () -> getMeasuredHeight().in(Meters));
    Logger.logNumber(this.getName() + "/leftHeight", () -> getLeftHeight().in(Meters));
    Logger.logNumber(this.getName() + "/rightHeight", () -> getRightHeight().in(Meters));

    Logger.logNumber(this.getName() + "/rawAbsolutePosition", absoluteEncoder::get);
    Logger.logBoolean(this.getName() + "/doneMoving", this::doneMoving);
    Logger.logNumber(this.getName() + "/cycleDelta", () -> getCycleDelta().in(Meters));
    // Logger.logNumber(this.getName() + "/offset", () -> encoderOffset);

    StatusChecks.Category statusChecks = StatusChecks.under(this);
    statusChecks.add("absoluteEncoderConnected", () -> absoluteEncoder.isConnected());
    statusChecks.add("absoluteEncoderUpdated", () -> absoluteEncoder.get() != 0.0);
  }

  public void setTargetHeight(Distance height) {
    targetHeight = clampHeight(height);
  }

  public Distance getTargetHeight() {
    return targetHeight;
  }

  public boolean isPastLimit() {
    return leftEncoder.getPosition() > maxHeight.in(Meters)
        || leftEncoder.getPosition() < minHeight.in(Meters)
        || rightEncoder.getPosition() > maxHeight.in(Meters)
        || rightEncoder.getPosition() < minHeight.in(Meters);
  }

  private Distance clampHeight(Distance height) {
    return Meters.of(MathUtil.clamp(height.in(Meters), minHeight.in(Meters), maxHeight.in(Meters)));
  }

  public boolean isHeightAchievable(Distance height) {
    return height.gt(minHeight) && height.lt(maxHeight);
  }

  private Distance getLeftHeight() {
    return Meters.of(leftEncoder.getPosition());
  }

  private Distance getRightHeight() {
    return Meters.of(rightEncoder.getPosition());
  }

  public Distance getMeasuredHeight() {
    return getLeftHeight().plus(getRightHeight()).div(2);
  }

  private Distance getCycleDelta() {
    return ratios.sensorToCarriage(Rotations.of(absoluteEncoder.get()));
  }

  public void moveUp() {
    if (needsToStop()) {
      stop();
      return;
    }

    leftMotor.set(0.10);
    rightMotor.set(0.10);
  }

  public void moveDown() {
    if (needsToStop()) {
      stop();
      return;
    }

    leftMotor.set(-0.10);
    rightMotor.set(-0.10);
  }

  public boolean doneMoving() {
    if (getTargetHeight() == null) return true;
    return debouncer.calculate(
        getMeasuredHeight().minus(targetHeight).abs(Meters) < tolerance.in(Meters));
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void run() {
    if (targetHeight == null) return; // If we havent set a target Height yet, do nothing

    if (!absoluteEncoder.isConnected()) {
      DriverStation.reportError("LinearController is missing encoder", true);

      stopMotors();
      return;
    }

    if (needsToStop()) {
      stop();
      return;
    }

    // Set onboard PID controller to follow
    leftPID.setReference(targetHeight.in(Meters), ControlType.kPosition, ClosedLoopSlot.kSlot0, kS);
    rightPID.setReference(
        targetHeight.in(Meters), ControlType.kPosition, ClosedLoopSlot.kSlot0, kS);
  }

  public LinearVelocity getMeasuredVelocity() {
    return MetersPerSecond.of(
      ratios.rotorRotationsToCarriageMeters() *
      (leftEncoder.getVelocity() + rightEncoder.getVelocity() / 2)
    );
  }

  private void stop() {
    if (getMeasuredVelocity().gt(MetersPerSecond.of(0))) {
      leftMotor.set(-1.0);
    } else {
      leftMotor.set(1.0);
    }
  }

  private boolean needsToStop() {
    LinearVelocity velocity = getMeasuredVelocity();

    LinearAcceleration gravityAcceleration;
    Distance currentDistance;

    if (velocity.lt(MetersPerSecond.of(0))) {
      gravityAcceleration = MetersPerSecondPerSecond.of(-9.81);
      currentDistance = getMeasuredHeight().minus(minHeight);
    } else {
      gravityAcceleration = MetersPerSecondPerSecond.of(9.81);
      currentDistance = maxHeight.minus(getMeasuredHeight());
    }

    return getSafeStopDistance(velocity, gravityAcceleration).lt(currentDistance);
  }

  private Distance getSafeStopDistance(LinearVelocity velocity, LinearAcceleration gravityAcceleration) {
    Force maxForce = ratios.motorToCarriage(NewtonMeters.of(motors.getTorque(40)));
    LinearAcceleration maxAcceleration = maxForce.div(carriageMass).plus(gravityAcceleration);

    Time stopTime = Seconds.of(velocity.in(MetersPerSecond) / maxAcceleration.in(MetersPerSecondPerSecond));
    Distance stopDistance = maxAcceleration.times(stopTime).times(stopTime).div(2);

    return stopDistance;
  } 

  @Override
  public void periodic() {}
}
