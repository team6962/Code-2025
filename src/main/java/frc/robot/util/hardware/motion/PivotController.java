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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
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
  private SparkMax motor;
  // Built-in relative NEO encoder
  private RelativeEncoder encoder;
  // Rev absolute through-bore encoder
  private DutyCycleEncoder absoluteEncoder;

  private Angle minAngle, maxAngle;

  public final Angle tolerance;

  private double encoderOffset = 0.0;

  private boolean reversed;

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

    this.motor = new SparkMax(motorCAN, MotorType.kBrushless);
    this.kS = kS;
    encoder = motor.getEncoder();
    this.encoderOffset = absolutePositionOffset;
    absoluteEncoder = new DutyCycleEncoder(absoluteEncoderDIO, 1.0, encoderOffset);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    pid = motor.getClosedLoopController();

    this.minAngle = minAngle;
    this.maxAngle = maxAngle;
    this.tolerance = tolerance;

    this.reversed = reversed;

    SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
    SparkMaxUtil.configureEncoder(motorConfig, 1.0 / gearing);
    SparkMaxUtil.configurePID(motorConfig, kP, kI, kD, 0.0, false);
    SparkMaxUtil.saveAndLog(this, motor, motorConfig);

    Logger.logMeasure(this.getName() + "/targetPosition", () -> getTargetAngle());
    Logger.logMeasure(this.getName() + "/position", () -> getPosition());
    Logger.logMeasure(
        this.getName() + "/relativePosition", () -> Rotations.of(encoder.getPosition()));
    Logger.logMeasure(
        this.getName() + "/rawAbsolutePosition", () -> Rotations.of(absoluteEncoder.get()));
    Logger.logBoolean(this.getName() + "/doneMoving", this::doneMoving);

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

    Logger.logNumber(this.getName() + "/minAngle", () -> minAngle.in(Rotations));
    Logger.logNumber(this.getName() + "/maxAngle", () -> maxAngle.in(Rotations));
    Logger.logNumber(this.getName() + "/targetAngle", () -> getTargetAngle().in(Rotations));
    Logger.logNumber(this.getName() + "/angle", () -> getPosition().in(Rotations));
    Logger.logNumber(this.getName() + "/relativePosition", () -> encoder.getPosition());
    Logger.logNumber(this.getName() + "/rawAbsolutePosition", () -> absoluteEncoder.get());
    Logger.logBoolean(this.getName() + "/doneMoving", this::doneMoving);
    Logger.logBoolean(this.getName() + "/forwardSafety", this::triggeredForwardSafety);
    Logger.logBoolean(this.getName() + "/reverseSafety", this::triggeredReverseSafety);
    // Logger.logBoolean(this.getName() + "/encoderConnected",  absoluteEncoder::isConnected);

    encoder.setPosition(getPosition().in(Rotations));
  }

  public void moveTowards(Angle requestedAngle) {
    if (requestedAngle == null) return; // If we havent set a target angle yet, do nothing
    targetAngle = clampAngle(requestedAngle);

    if (!absoluteEncoder.isConnected()) {
      motor.stopMotor();
      return;
    }

    encoder.setPosition(getPosition().in(Rotations));

    // Set onboard PID controller to follow

    if (RobotBase.isSimulation())
      sim.setInputVoltage(simPID.calculate(sim.getAngleRads(), targetAngle.in(Radians)) * 12.0);

    // System.out.println(Math.signum(targetAngle.getRadians() - getPosition().getRadians()));
    // System.out.println("kS: " + kS);
    // System.out.println(feedforward.calculate(setpointState.position, setpointState.velocity));

    if (canMoveInDirection(targetAngle.minus(getPosition()).in(Rotations))) {
      pid.setReference(
          targetAngle.in(Rotations),
          ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          Math.cos(MANIPULATOR_PIVOT.CENTER_OF_MASS_OFFSET.in(Radians) + getPosition().in(Radians))
              * kS);
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
    return runOnce(() -> moveSpeed(speed));
  }

  public void moveSpeed(double speed) {
    if (canMoveInDirection(speed)) motor.set(speed);
    else motor.set(0);
  }

  public void moveUp() {
    moveSpeed(0.05);
  }

  public void moveDown() {
    moveSpeed(-0.05);
  }

  public Angle getPosition() {
    if (Robot.isSimulation()) return Radians.of(sim.getAngleRads());

    double factor = reversed ? -1 : 1;

    // map from 0 - 1 rotations to -0.5 to 0.5 rotations, where 0 is straight
    // out
    double absoluteAngle = absoluteEncoder.get() * factor; // rotations

    // keeps the range between 0 and 1
    if (absoluteAngle < 0) absoluteAngle++;
    absoluteAngle %= 1.0;

    // wrap at 0.5 rotations
    if (absoluteAngle > 0.5) {
      absoluteAngle -= 1;
    }

    return Rotations.of(absoluteAngle);
  }

  public boolean canMoveInDirection(double velocity) {
    if (velocity > 0.0) return getPosition().lt(maxAngle);
    if (velocity < 0.0) return getPosition().gt(minAngle);
    return true;
  }

  public boolean triggeredForwardSafety() {
    return getPosition().gt(maxAngle) && encoder.getVelocity() > 0.0;
  }

  public boolean triggeredReverseSafety() {
    return getPosition().lt(minAngle) && encoder.getVelocity() < -0.0;
  }

  public boolean doneMoving() {
    if (getTargetAngle() == null) return true;
    return debouncer.calculate(
        getPosition().minus(targetAngle).abs(Radians) < tolerance.in(Radians));
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    encoder.setPosition(getPosition().in(Rotations));

    if (triggeredForwardSafety() || triggeredReverseSafety()) {
      stopMotor();
    }
  }
}
