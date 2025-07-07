package frc.robot.util.hardware.motion;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;

import com.ctre.phoenix6.controls.VoltageOut;
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
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.util.hardware.SparkMaxUtil;

/*
 * Uses oboard 1kHz PID, Feedforward, and Trapazoidal Profiles to
 * control a pivot mechanism precisely, smoothly, and accurately
 */

public class DualLinearActuator extends SubsystemBase {
  private Distance targetHeight = Meters.of(0.0);
  private double kG = 0.0;

  protected SparkMax leftMotor, rightMotor;
  private RelativeEncoder leftEncoder, rightEncoder;
  private SparkClosedLoopController leftPID, rightPID;
  private DigitalInput ceilingLimit, floorLimit;
  private Distance baseHeight, minHeight, maxHeight, tolerance;
  private boolean moving = false;

  private Debouncer debouncer = new Debouncer(0.1);

  private Distance spoolHeight;

  private boolean zeroed = false;

  /**
   * Constructs a new DualLinearController.
   *
   * @param leftCAN The CAN ID for the left motor controller.
   * @param rightCAN The CAN ID for the right motor controller.
   * @param absoluteEncoderDIO The DIO port for the absolute encoder.
   * @param absolutePositionOffset The offset for the absolute encoder position.
   * @param kP The proportional gain for the PID controller.
   * @param kG The static gain for the PID controller.
   * @param sensorToMotorRatio The ratio of the sensor to motor.
   * @param spoolHeight The ratio of the mechanism to sensor.
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
      double kG,
      double gearing,
      Distance spoolHeight,
      Distance baseHeight,
      Distance minHeight,
      Distance maxHeight,
      Distance tolerance) {
    setName(name);

    this.kG = kG;
    this.baseHeight = baseHeight;
    this.minHeight = minHeight;
    this.maxHeight = maxHeight;
    this.tolerance = tolerance;
    this.spoolHeight = spoolHeight;

    leftMotor = new SparkMax(leftCAN, MotorType.kBrushless);
    rightMotor = new SparkMax(rightCAN, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
    leftPID = leftMotor.getClosedLoopController();
    rightPID = rightMotor.getClosedLoopController();

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.minOutput((-12. + kG) / 12.);
    SparkMaxUtil.configure(motorConfig, true, IdleMode.kBrake, 80, 60);
    SparkMaxUtil.configureEncoder(motorConfig, spoolHeight.in(Meters) / gearing);
    SparkMaxUtil.configurePID(
        motorConfig, kP, 0.0, 0.0, minHeight.in(Meters), maxHeight.in(Meters), false);
    SparkMaxUtil.saveAndLog(this, leftMotor, motorConfig);

    motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.minOutput((-12. + kG) / 12.);
    SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake, 80, 60);
    SparkMaxUtil.configureEncoder(motorConfig, spoolHeight.in(Meters) / gearing);
    SparkMaxUtil.configurePID(
        motorConfig, kP, 0.0, 0.0, minHeight.in(Meters), maxHeight.in(Meters), false);
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

    Logger.logBoolean(this.getName() + "/limits/ceil", ceilingLimit::get);
    Logger.logBoolean(this.getName() + "/limits/floor", () -> !floorLimit.get());

    Logger.logMeasure(
        this.getName() + "/motors/left/current", () -> Amps.of(leftMotor.getOutputCurrent()));
    Logger.logNumber(this.getName() + "/motors/left/dutycycle", () -> leftMotor.getAppliedOutput());

    Logger.logMeasure(
        this.getName() + "/motors/right/current", () -> Amps.of(rightMotor.getOutputCurrent()));
    Logger.logNumber(
        this.getName() + "/motors/right/dutycycle", () -> rightMotor.getAppliedOutput());
    Logger.logBoolean(this.getName() + "/moving", () -> moving);

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
    return move(-0.1);
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

  public boolean atPosition(Distance height) {
    if (height == null) return true;
    return debouncer.calculate(getAverageHeight().minus(height).abs(Meters) < tolerance.in(Meters));
  }

  public boolean doneMoving() {
    if (atPosition(targetHeight)) {
      moving = false;
      return true;
    }
    return false;
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void moveToImmediate(Distance requestedHeight) {
    targetHeight = clampHeight(requestedHeight);
    if (targetHeight == null) return; // If we havent set a target Height yet, do nothing

    if (!canMoveInDirection(requestedHeight.minus(getAverageHeight()).in(Meters))) {
      if (!triggeredFloorLimit() && triggeredCeilingLimit()) {
        leftMotor.setVoltage(kG);
        rightMotor.setVoltage(kG);

        return;
      }

      stopMotors();
      return;
    }

    moving = true;
    leftPID.setReference(targetHeight.in(Meters), ControlType.kPosition, ClosedLoopSlot.kSlot0, kG);
    rightPID.setReference(
        targetHeight.in(Meters), ControlType.kPosition, ClosedLoopSlot.kSlot0, kG);
  }

  public boolean triggeredCeilingLimit() {
    if (!zeroed) {
      return true;
    }
    return ceilingLimit.get();
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

  double appliedVoltage;

  @Override
  public void periodic() {
    if (triggeredFloorLimit()) {
      seedEncoders(baseHeight);
    }

    if (appliedVoltage != 0 && !canMoveInDirection(appliedVoltage)) {
      stopMotors();
      Logger.log(getName() + "/stopped", "backup");
      if (getCurrentCommand() != null) getCurrentCommand().cancel();
      return;
    }
  }

  public Command _setHeight(Distance height) {
    return this.run(() -> moveToImmediate(height)).until(this::doneMoving);
  }

  public Command _hold() {
    return Commands.defer(
        () -> {
          Distance position = getAverageHeight();

          return this.run(() -> moveToImmediate(position));
        },
        Set.of(this));
  }

  private SysIdRoutine createRoutine(String name) {
    return new SysIdRoutine(
      new SysIdRoutine.Config(Volts.per(Second).of(8.0), Volts.of(2), Seconds.of(0.8)),
      new SysIdRoutine.Mechanism(
        voltage -> {
          if (voltage.in(Volts) != 0 && !canMoveInDirection(voltage.in(Volts))) {
            stopMotors();
            Logger.log(getName() + "/stopped", "yes");
            return;
          }

          Logger.log(getName() + "/stopped", "no");

          appliedVoltage = voltage.in(Volts);

          leftMotor.setVoltage(voltage.in(Volts));
          rightMotor.setVoltage(voltage.in(Volts));
        },
        log ->
          log.motor("elevator-" + name)
            .voltage(Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage()))
            .linearPosition(getLeftHeight()).linearVelocity(MetersPerSecond.of(leftEncoder.getVelocity())),
        this
      )
    );
  }

  public Command sysId() {
    Time timeout = Seconds.of(2.0);

    // BOTH DIRECTIONS - DO NOT USE
    // return Commands.sequence(
    //   _hold().withTimeout(timeout),
    //   _setHeight(minHeight.plus(Inches.of(5))),
    //   _hold().withTimeout(timeout),
    //   calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
    //   _hold().withTimeout(timeout),
    //   _setHeight(maxHeight.minus(Inches.of(5))),
    //   _hold().withTimeout(timeout),
    //   calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
    //   _hold().withTimeout(timeout),
    //   _setHeight(minHeight.plus(Inches.of(5))),
    //   _hold().withTimeout(timeout),
    //   calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
    //   _hold().withTimeout(timeout),
    //   _setHeight(maxHeight.minus(Inches.of(5))),
    //   _hold().withTimeout(timeout),
    //   calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
    //   _hold().withTimeout(timeout)
    // );

    SysIdRoutine forwardRoutine = createRoutine("forward");
    SysIdRoutine reverseRoutine = createRoutine("reverse");

    return Commands.sequence(
      _hold().withTimeout(timeout),
      _setHeight(minHeight.plus(Inches.of(5))),
      _hold().withTimeout(timeout),
      forwardRoutine.quasistatic(SysIdRoutine.Direction.kForward),
      _hold().withTimeout(timeout),
      _setHeight(minHeight.plus(Inches.of(5))),
      _hold().withTimeout(timeout),
      forwardRoutine.dynamic(SysIdRoutine.Direction.kForward),
      _hold().withTimeout(timeout),
      _setHeight(minHeight.plus(Inches.of(5))),
      _hold().withTimeout(timeout),
      forwardRoutine.quasistatic(SysIdRoutine.Direction.kForward),
      _hold().withTimeout(timeout),
      _setHeight(minHeight.plus(Inches.of(5))),
      _hold().withTimeout(timeout),
      forwardRoutine.dynamic(SysIdRoutine.Direction.kForward)
      // _hold().withTimeout(timeout),
      // _setHeight(maxHeight.minus(Inches.of(5))),
      // _hold().withTimeout(timeout),
      // reverseRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
      // _hold().withTimeout(timeout),
      // _setHeight(maxHeight.minus(Inches.of(5))),
      // _hold().withTimeout(timeout),
      // reverseRoutine.dynamic(SysIdRoutine.Direction.kReverse),
      // _hold().withTimeout(timeout)
    );
  }
}
