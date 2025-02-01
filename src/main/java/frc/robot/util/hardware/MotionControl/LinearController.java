package frc.robot.util.hardware.MotionControl;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.hardware.SparkMaxUtil;

/*
 * Uses oboard 1kHz PID, Feedforward, and Trapazoidal Profiles to
 * control a pivot mechanism precisely, smoothly, and accurately
 */

public class LinearController {
  private Distance targetHeight = null;
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

  private Distance minHeight, maxHeight, tolerance;

  private double encoderOffset = 0.0;

  private boolean reversed;

  private Debouncer debouncer = new Debouncer(0.1);

  private Distance achievableHeight = Meters.of(0.0);

  private SingleJointedArmSim sim;

  private PIDController simPID;

  public LinearController(
      SubsystemBase subsystem,
      SparkMax motor,
      int absoluteEncoderDIO,
      double absolutePositionOffset,
      double kP,
      double kS,
      double gearing,
      Distance minHeight,
      Distance maxHeight,
      Distance tolerance,
      boolean reversed) {
    // feedforward = new ArmFeedforward(kS, 0.0, 0.0, 0.0);
    // profile = new TrapezoidProfile(
    //   new Constraints(maxVelocity, maxAcceleration)
    // );
    this.kS = kS;
    encoder = motor.getEncoder();
    absoluteEncoder = new DutyCycleEncoder(absoluteEncoderDIO, gearing, encoderOffset);
    pid = motor.getClosedLoopController();

    SparkMaxConfig motorConfig = new SparkMaxConfig();

    this.motor = motor;
    this.minHeight = minHeight;
    this.maxHeight = maxHeight;
    this.tolerance = tolerance;

    this.reversed = reversed;
    encoderOffset = absolutePositionOffset;
    SparkMaxUtil.configureEncoder(motorConfig, 2.0 * Math.PI / gearing);
    SparkMaxUtil.configurePID(motorConfig, kP, 0.0, 0.0, 0.0, false);

    Logger.logNumber(subsystem.getName() + "/targetHeight", () -> getTargetHeight().in(Meters));
    Logger.logNumber(subsystem.getName() + "/position", () -> getHeight().in(Meters));
    Logger.logNumber(subsystem.getName() + "/relativePosition", () -> encoder.getPosition());
    Logger.logNumber(
        subsystem.getName() + "/rawAbsolutePosition",
        () -> Rotations.of(absoluteEncoder.get()).in(Radians));
    Logger.logBoolean(subsystem.getName() + "/doneMoving", this::doneMoving);

    StatusChecks.Category statusChecks = StatusChecks.under(subsystem);
    statusChecks.add("absoluteEncoderConnected", () -> absoluteEncoder.isConnected());
    statusChecks.add("absoluteEncoderUpdated", () -> absoluteEncoder.get() != 0.0);

    // sim = new SingleJointedArmSim(
    //   DCMotor.getNEO(1),
    //   gearing,
    //   SingleJointedArmSim.estimateMOI(1.0, 5.0),
    //   1.0,
    //   Units.degreesToRadians(-180.0),
    //   Units.degreesToRadians(180.0),
    //   false,
    //   0.0
    // );
    // simPID = new PIDController(kP, 0, 0);
  }

  public void run() {
    if (targetHeight == null) return; // If we havent set a target Height yet, do nothing
    if (!absoluteEncoder.isConnected()) {
      motor.stopMotor();
      return;
    }

    encoder.setPosition(getHeight().in(Meters));

    setAchievableHeight();

    if (doneMoving()) {
      motor.stopMotor();
      if (RobotBase.isSimulation()) sim.setInputVoltage(0.0);
      return;
    }

    // Set onboard PID controller to follow
    pid.setReference(achievableHeight.in(Meters), ControlType.kPosition, ClosedLoopSlot.kSlot0, kS);

    // System.out.println(Math.signum(targetHeight.getRadians() - getHeight().getRadians()));
    // System.out.println("kS: " + kS);
    // System.out.println(feedforward.calculate(setpointState.position, setpointState.velocity));

    if (motor.getAppliedOutput() > 0.0 && getHeight().gt(maxHeight)) {
      motor.stopMotor();
    }

    if (motor.getAppliedOutput() < 0.0 && getHeight().lt(maxHeight)) {
      motor.stopMotor();
    }
  }

  public void setTargetHeight(Distance height) {
    targetHeight = height;
    setAchievableHeight();
  }

  public Distance getTargetHeight() {
    return targetHeight;
  }

  public boolean isPastLimit() {
    return encoder.getPosition() > maxHeight.in(Meters)
        || encoder.getPosition() < minHeight.in(Meters);
  }

  private void setAchievableHeight() {
    achievableHeight = targetHeight;
    if (achievableHeight.lt(minHeight)) {
      achievableHeight = minHeight;
    } else if (achievableHeight.gt(maxHeight)) {
      achievableHeight = maxHeight;
    }
  }

  public boolean isHeightAchievable(Distance height) {
    return height.gt(minHeight) && height.lt(maxHeight);
  }

  public Distance getHeight() {
    double factor = 1;
    if (reversed) {
      factor = -1;
    }

    // ((0.26934 + x) * -1)

    // Map absolute encoder position from 0 - 1 rotations to -pi - pi radians, where 0 is straight
    // out
    double absoluteHeight = (absoluteEncoder.get() + encoderOffset) * factor;

    return Meters.of(absoluteHeight);
  }

  public boolean doneMoving() {
    if (getTargetHeight() == null) return true;
    return debouncer.calculate(
        getHeight().minus(achievableHeight).abs(Meters) < tolerance.in(Meters));
  }

  public void setMaxHeight(Distance newMaxHeight) {
    maxHeight = newMaxHeight;
  }
}
