package frc.robot.util.hardware.MotionControl;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.hardware.SparkMaxUtil;

/*
 * Uses oboard 1kHz PID, Feedforward, and Trapazoidal Profiles to
 * control a pivot mechanism precisely, smoothly, and accurately
 */

public class PivotController extends SubsystemBase{
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
      int motorCAN,
      int absoluteEncoderDIO,
      double absolutePositionOffset,
      double kP,
      double kS,
      double gearing,
      Angle minAngle,
      Angle maxAngle,
      Angle tolerance,
      boolean reversed) {
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
    SparkMaxUtil.configurePID(motorConfig, kP, 0.0, 0.0, 0.0, false);
    SparkMaxUtil.saveAndLog(this, motor, motorConfig);

    Logger.logMeasure(this.getName() + "/targetPosition", () -> getTargetAngle());
    Logger.logMeasure(this.getName() + "/position", () -> getPosition());
    Logger.logMeasure(
        this.getName() + "/relativePosition",
        () -> Rotations.of(encoder.getPosition()));
    Logger.logMeasure(
        this.getName() + "/rawAbsolutePosition",
        () -> Rotations.of(absoluteEncoder.get()));
    Logger.logBoolean(this.getName() + "/doneMoving", this::doneMoving);

    StatusChecks.Category statusChecks = StatusChecks.under(this);
    statusChecks.add("absoluteEncoderConnected", () -> absoluteEncoder.isConnected());
    statusChecks.add("absoluteEncoderUpdated", () -> absoluteEncoder.get() != 0.0);

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



    Logger.logNumber(this.getName() + "/targetAngle", () -> getTargetAngle().in(Rotations));
    Logger.logNumber(this.getName() + "/angle", () -> getPosition().in(Rotations));
    Logger.logNumber(
        this.getName() + "/relativePosition",
        () -> Rotations.of(encoder.getPosition()).in(Rotations));
    Logger.logNumber(
        this.getName() + "/rawAbsolutePosition",
        () -> Rotations.of(absoluteEncoder.get()).in(Rotations));
    Logger.logBoolean(this.getName() + "/encoderConnected",  () -> absoluteEncoder.isConnected());
    Logger.logBoolean(this.getName() + "/doneMoving", this::doneMoving);
    Logger.log(this.getName() + "/minAngle", minAngle.in(Rotations));
    Logger.log(this.getName() + "/maxAngle", maxAngle.in(Rotations));
    Logger.logNumber(this.getName()+"/appliedOutput", () -> motor.getAppliedOutput());
  }

  public void run() {
    if (targetAngle == null) return; // If we havent set a target angle yet, do nothing
    if (!absoluteEncoder.isConnected()) {
      motor.stopMotor();
      return;
    }

    // System.out.println(getPosition().getDegrees());

    // Re-seed the relative encoder with the absolute encoder when not moving
    // if (doneMoving()) {
    encoder.setPosition(getPosition().in(Rotations));
    // }

    // if (setpointState == null) {
    //   setpointState = new State(getAbsolutePosition().getRadians(), getVelocity().getRadians());
    // };

    // Calculate the setpoint following a trapazoidal profile (smooth ramp up and down acceleration
    // curves)
    // State targetState = new State(targetAngle.getRadians(), 0.0);

    // setpointState = profile.calculate(Robot.getLoopTime(), setpointState, targetState);

    if (doneMoving()) {
      motor.stopMotor();
      if (RobotBase.isSimulation()) sim.setInputVoltage(0.0);
      return;
    }

    // Set onboard PID controller to follow

    if (RobotBase.isSimulation())
      sim.setInputVoltage(simPID.calculate(sim.getAngleRads(), targetAngle.in(Radians)) * 12.0);

    // System.out.println(Math.signum(targetAngle.getRadians() - getPosition().getRadians()));
    // System.out.println("kS: " + kS);
    // System.out.println(feedforward.calculate(setpointState.position, setpointState.velocity));


    // if (getPosition().gt(maxAngle) && encoder.getVelocity() > 0.0) {
    //   motor.stopMotor();
    //   if (RobotBase.isSimulation()) sim.setInputVoltage(0.0);
    //   return;
    // }

    // if (getPosition().lt(minAngle) && encoder.getVelocity() < -0.0) {
    //   motor.stopMotor();
    //   if (RobotBase.isSimulation()) sim.setInputVoltage(0.0);
    //   return;
    // }

    pid.setReference(targetAngle.in(Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot0, kS);

    if (Robot.isSimulation()) sim.update(Robot.getLoopTime());
  }

  public Angle clampAngle (Angle angle){
    if (angle.gt(maxAngle)) return maxAngle;
    if (angle.lt(minAngle)) return minAngle;
    return angle;
  }

  public void setTargetAngle(Angle angle) {
    targetAngle = clampAngle(angle);
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

  public void moveUp(){
    motor.set(0.05);
  }

  public void moveDown(){
    motor.set(-0.05);
  }

  public boolean isInRange(Angle angle) {
    return angle.gt(minAngle) && angle.lt(maxAngle);
  }

  public Angle getPosition() {
    if (Robot.isSimulation()) return Radians.of(sim.getAngleRads());

    double factor = reversed ? -1 : 1;

    // ((0.26934 + x) * -1)

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
      
  }
}
