package frc.robot.util.hardware.MotionControl;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.hardware.SparkMaxUtil;

/*
 * Uses oboard 1kHz PID, Feedforward, and Trapazoidal Profiles to
 * control a pivot mechanism precisely, smoothly, and accurately
 */


public class DualLinearController extends SubsystemBase {
  private Distance targetHeight = Meters.of(0.0);
  private double kS = 0.0;

  private SparkMax leader, follower;
  private RelativeEncoder leaderEncoder, followerEncoder;
  private SparkClosedLoopController leaderPID, followerPID;

  private DutyCycleEncoder absoluteEncoder;

  private Distance minHeight, maxHeight, tolerance;

  private double encoderOffset = 0.0;

  private Debouncer debouncer = new Debouncer(0.1);

  private Distance achievableHeight = Meters.of(0.0);


  /**
   * Constructs a new DualLinearController.
   *
   * @param leaderCAN The CAN ID for the leader motor controller.
   * @param followerCAN The CAN ID for the follower motor controller.
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
  public DualLinearController(
      int leaderCAN,
      int followerCAN,
      int absoluteEncoderDIO,
      double absolutePositionOffset,
      double kP,
      double kS,
      double gearing,
      double mechanismToSensor,
      Distance minHeight,
      Distance maxHeight,
      Distance tolerance) {

    this.kS = kS;
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    leader = new SparkMax(leaderCAN, MotorType.kBrushless);
    follower = new SparkMax(followerCAN, MotorType.kBrushless);
    leaderEncoder = leader.getEncoder();
    followerEncoder = follower.getEncoder();
    leaderPID = leader.getClosedLoopController();
    followerPID = follower.getClosedLoopController();
    absoluteEncoder =
        new DutyCycleEncoder(absoluteEncoderDIO, mechanismToSensor, encoderOffset);

    this.minHeight = minHeight;
    this.maxHeight = maxHeight;
    this.tolerance = tolerance;

    encoderOffset = absolutePositionOffset;

    SparkMaxUtil.configure(motorConfig, true, IdleMode.kBrake);
    SparkMaxUtil.configureEncoder(motorConfig, mechanismToSensor / gearing);
    SparkMaxUtil.configurePID(motorConfig, kP, 0.0, 0.0, 0.0, false);
    SparkMaxUtil.saveAndLog(this, leader, motorConfig);

    SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
    SparkMaxUtil.configureEncoder(motorConfig, mechanismToSensor / gearing);
    SparkMaxUtil.configurePID(motorConfig, kP, 0.0, 0.0, 0.0, false);
    SparkMaxUtil.saveAndLog(this, follower, motorConfig);

    Logger.logNumber(this.getName() + "/targetHeight", () -> getTargetHeight().in(Meters));
    Logger.logNumber(this.getName() + "/position", () -> getHeight().in(Meters));
    Logger.logNumber(this.getName() + "/relativePosition", () -> leaderEncoder.getPosition());
    Logger.logNumber(
        this.getName() + "/rawAbsolutePosition",
        () -> Rotations.of(absoluteEncoder.get()).in(Radians));
    Logger.logBoolean(this.getName() + "/doneMoving", this::doneMoving);

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
    return leaderEncoder.getPosition() > maxHeight.in(Meters)
        || leaderEncoder.getPosition() < minHeight.in(Meters)
        || followerEncoder.getPosition() > maxHeight.in(Meters)
        || followerEncoder.getPosition() < minHeight.in(Meters);
  }

  private Distance clampHeight(Distance height) {
    return Meters.of(
        MathUtil.clamp(achievableHeight.in(Meters), minHeight.in(Meters), maxHeight.in(Meters)));
  }

  public boolean isHeightAchievable(Distance height) {
    return height.gt(minHeight) && height.lt(maxHeight);
  }

  public Distance getHeight() {
    return Meters.of(absoluteEncoder.get());
  }

  public boolean doneMoving() {
    if (getTargetHeight() == null) return true;
    return debouncer.calculate(
        getHeight().minus(achievableHeight).abs(Meters) < tolerance.in(Meters));
  }

  public void stopMotors() {
    leader.stopMotor();
    follower.stopMotor();
  }

  public void run() {
    if (targetHeight == null) return; // If we havent set a target Height yet, do nothing

    if (!absoluteEncoder.isConnected()) {
      stopMotors();
      return;
    }

    if (leader.getFaults() != null || follower.getFaults() != null) {
      stopMotors();
      return;
    }

    leaderEncoder.setPosition(getHeight().in(Meters));
    followerEncoder.setPosition(getHeight().in(Meters));

    if (doneMoving()) {
      stopMotors();
      return;
    }
    // Set onboard PID controller to follow
    leaderPID.setReference(
        achievableHeight.in(Meters), ControlType.kPosition, ClosedLoopSlot.kSlot0, kS);
    followerPID.setReference(
        achievableHeight.in(Meters), ControlType.kPosition, ClosedLoopSlot.kSlot0, kS);

    if (leader.getAppliedOutput() > 0.0 && getHeight().gt(maxHeight)) {
      stopMotors();
    }

    if (leader.getAppliedOutput() < 0.0 && getHeight().lt(minHeight)) {
      stopMotors();
    }

    if (follower.getAppliedOutput() > 0.0 && getHeight().gt(maxHeight)) {
      stopMotors();
    }

    if (follower.getAppliedOutput() < 0.0 && getHeight().lt(minHeight)) {
      stopMotors();
    }
  }

  @Override
  public void periodic() {}
}
