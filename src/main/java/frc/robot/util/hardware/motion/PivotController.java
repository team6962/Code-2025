package frc.robot.util.hardware.motion;

import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.MANIPULATOR_PIVOT;
import frc.robot.Robot;
import frc.robot.util.hardware.SparkMaxUtil;

/*
 * Uses oboard 1kHz PID, Feedforward, and Trapazoidal Profiles to
 * control a pivot mechanism precisely, smoothly, and accurately
 */

public class PivotController extends SubsystemBase {
  private Angle targetAngle = Degrees.of(0);
  // State setpointState;
  private double kS = 0.0;
  // Onboard spark max PID controller. Runs at 1kHz
  private SparkClosedLoopController pid;
  // CAN Spark Max motor controller;
  protected SparkMax motor;
  // Built-in relative NEO encoder
  protected RelativeEncoder encoder;
  // Rev absolute through-bore encoder
  protected DutyCycleEncoder absoluteEncoder;

  private Angle minAngle, maxAngle;

  public final Angle tolerance;

  private final boolean reversed;

  private Debouncer debouncer = new Debouncer(0.1);

  private SingleJointedArmSim sim;

  private PIDController simPID;

  public PivotController(
      String name,
      int motorCAN,
      int absoluteEncoderDIO,
      double absolutePositionOffset,
      double kP,
      double kI,
      double kD,
      double kS,
      double gearing,
      Angle minAngle,
      Angle maxAngle,
      Angle tolerance,
      boolean reversed) {
    
    setName(name);
    // feedforward = new ArmFeedforward(kS, 0.0, 0.0, 0.0);
    // profile = new TrapezoidProfile(
    //   new Constraints(maxVelocity, maxAcceleration)
    // );

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motor = new SparkMax(motorCAN, MotorType.kBrushless);
    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();

    absoluteEncoder = new DutyCycleEncoder(absoluteEncoderDIO, 1.0, absolutePositionOffset);

    this.kS = kS;
    this.minAngle = minAngle;
    this.maxAngle = maxAngle;
    this.tolerance = tolerance;

    this.reversed = reversed;

    SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
    SparkMaxUtil.configureEncoder(motorConfig, 1.0 / gearing);
    SparkMaxUtil.configurePID(motorConfig, kP, kI, kD, 0.0, minAngle.in(Rotations), maxAngle.in(Rotations), false);
    SparkMaxUtil.saveAndLog(this, motor, motorConfig);

    StatusChecks.Category statusChecks = StatusChecks.under(this);
    statusChecks.add("absoluteEncoderConnected", () -> absoluteEncoder.isConnected());
    statusChecks.add("absoluteEncoderUpdated", () -> absoluteEncoder.get() != 0.0);
    statusChecks.add("motor", motor);

    sim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            gearing,
            SingleJointedArmSim.estimateMOI(1.0, 5.0),
            1.0,
            Units.degreesToRadians(-180.0),
            Units.degreesToRadians(180.0),
            false,
            0.0);
    simPID = new PIDController(kP, 0, 0);

    
    Logger.logBoolean(this.getName() + "/doneMoving", this::doneMoving);
    Logger.logBoolean(this.getName() + "/safety/forward", this::triggeredForwardSafety);
    Logger.logBoolean(this.getName() + "/safety/reverse", this::triggeredReverseSafety);
    Logger.logNumber(this.getName() + "/appliedDutyCycle", () -> motor.getAppliedOutput());

    Logger.logNumber(this.getName() + "/angle/target", () -> getTargetAngle().in(Rotations));
    Logger.logNumber(this.getName() + "/angle/relative", () -> getRelativePosition().in(Rotations));
    Logger.logNumber(this.getName() + "/angle/absolute", () -> getAbsolutePosition().in(Rotations));
    Logger.logNumber(this.getName() + "/angle/raw", () -> getRawAbsolutePosition().in(Rotations));
    Logger.logNumber(this.getName() + "/angle/min", () -> getMinAngle().in(Rotations));
    Logger.logNumber(this.getName() + "/angle/max", () -> getMaxAngle().in(Rotations));

    Logger.logBoolean(this.getName() + "/encoderConnected",  absoluteEncoder::isConnected);

    seedEncoder();
  }

  public void seedEncoder() {
    Logger.log(this.getName() + "/lastSeeded", Timer.getFPGATimestamp());
    encoder.setPosition(getAbsolutePosition().in(Rotations));
    System.out.println("AFTA::::::::::::::" + getRelativePosition());
  }

  public void moveTowards(Angle requestedAngle) {
    if (requestedAngle == null) return; // If we havent set a target angle yet, do nothing
    targetAngle = clampAngle(requestedAngle);

    if (!absoluteEncoder.isConnected()) {
      motor.stopMotor();
      return;
    }

    // encoder.setPosition(getPosition().in(Rotations));

    // Set onboard PID controller to follow

    if (RobotBase.isSimulation())
      sim.setInputVoltage(simPID.calculate(sim.getAngleRads(), targetAngle.in(Radians)) * 12.0);

    // System.out.println(Math.signum(targetAngle.getRadians() - getPosition().getRadians()));
    // System.out.println("kS: " + kS);
    // System.out.println(feedforward.calculate(setpointState.position, setpointState.velocity));

    // if (targetAngle.gt(Degrees.of(90).minus(MANIPULATOR_PIVOT.CENTER_OF_MASS_OFFSET))) {
    //   targetAngle = targetAngle.minus(MANIPULATOR_PIVOT.BACKLASH);
    // } else {
    //   targetAngle = targetAngle.plus(MANIPULATOR_PIVOT.BACKLASH);
    // }

    if (canMoveInDirection(targetAngle.minus(getRelativePosition()).in(Rotations))) {
      pid.setReference(
          targetAngle.in(Rotations),
          ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          calculateKS(getAbsolutePosition())
      );
    } else {
      motor.stopMotor();
    }

    // pid.setReference(achievableAngle.in(Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot0,
    // kS);
    // pid.setReference(achievableAngle.in(Rotations), ControlType.kPosition);]

    if (Robot.isSimulation()) sim.update(Robot.getLoopTime());
  }

  public Angle clampAngle(Angle angle) {
    if (angle.gt(maxAngle)) return maxAngle;
    if (angle.lt(minAngle)) return minAngle;
    return angle;
  }

  public Angle getTargetAngle() {
    return targetAngle;
  }

  public void setMaxAngle(Angle maxAngle) {
    this.maxAngle = maxAngle;
    targetAngle = clampAngle(targetAngle);
  }

  public Angle getMaxAngle() {
    return maxAngle;
  }

  public void setMinAngle(Angle minAngle) {
    this.minAngle = minAngle;
    targetAngle = clampAngle(targetAngle);
  }

  public Angle getMinAngle() {
    return minAngle;
  }

  public void setMinMaxAngle(Angle minAngle, Angle maxAngle) {
    this.minAngle = minAngle;
    this.maxAngle = maxAngle;
    targetAngle = clampAngle(targetAngle);
  }

  public Command move(double speed) {
    return run(() -> moveSpeed(speed)).finallyDo(motor::stopMotor);
  }

  public void moveSpeed(double speed) {
    // if (canMoveInDirection(speed))
    motor.set(speed);
    // else motor.set(0);
  }

  public Command up() {
    return move(0.1);
  }

  public Command down() {
    return move(-0.1);
  }

  public double calculateKS(Angle currentAngle) {
    return kS * Math.cos(MANIPULATOR_PIVOT.CENTER_OF_MASS_OFFSET.in(Radians) + currentAngle.in(Radians));
  }

  private Angle wrapAngle(Angle angle) {
    double factor = reversed ? -1 : 1;

    // map from 0 - 1 rotations to -0.5 to 0.5 rotations, where 0 is straight
    // out
    double absoluteAngle = angle.in(Rotations) * factor; // rotations

    // keeps the range between 0 and 1
    if (absoluteAngle < 0) absoluteAngle++;
    absoluteAngle %= 1.0;

    // wrap at 0.5 rotations
    if (absoluteAngle > 0.5) {
      absoluteAngle -= 1;
    }

    return Rotations.of(absoluteAngle);
  }

  public Angle getRelativePosition() {
    return Rotations.of(encoder.getPosition());

    // if (Robot.isSimulation()) return Radians.of(sim.getAngleRads());
  }

  public Angle getRawAbsolutePosition() {
    return Rotations.of(absoluteEncoder.get());
  }

  public Angle getAbsolutePosition() {
    return wrapAngle(getRawAbsolutePosition());
  }

  public boolean canMoveInDirection(double velocity) {
    if (velocity > 0.0) return getAbsolutePosition().lt(maxAngle);
    if (velocity < 0.0) return getAbsolutePosition().gt(minAngle);
    return true;
  }

  public boolean triggeredForwardSafety() {
    return getAbsolutePosition().gt(maxAngle) && encoder.getVelocity() > 0.0;
  }

  public boolean triggeredReverseSafety() {
    return getAbsolutePosition().lt(minAngle) && encoder.getVelocity() < -0.0;
  }

  public boolean doneMoving() {
    if (getTargetAngle() == null) return true;
    return debouncer.calculate(
        getRelativePosition().minus(targetAngle).abs(Radians) < tolerance.in(Radians));
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    seedEncoder();

    if (triggeredForwardSafety() || triggeredReverseSafety()) {
      stopMotor();
    }
  }
}
