package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;
import com.team6962.lib.utils.CTREUtils;
import com.team6962.lib.utils.MeasureMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Consumer;

/** A swerve module, consisting of a drive motor, a steer motor, and a steer encoder. */
public class SwerveModule extends SubsystemBase implements AutoCloseable {
  /** The drive motor for this module. Works in rotor units. */
  private TalonFX driveMotor;

  /** The steer motor for this module. Works in mechanism units. */
  private TalonFX steerMotor;

  /**
   * The steer encoder for this module, whose position is directly lined up with the wheel. This
   * means that the encoder outputs values in mechanism rotations.
   */
  private CANcoder steerEncoder;

  private SwerveConfig constants;
  private SwerveModule.Corner corner;

  private StatusSignal<Angle> drivePositionIn;
  private StatusSignal<Angle> steerAngleIn;
  private StatusSignal<AngularVelocity> driveSpeedIn;
  private StatusSignal<AngularVelocity> steerVelocityIn;
  private StatusSignal<AngularAcceleration> driveAccelerationIn;
  private StatusSignal<AngularAcceleration> steerAccelerationIn;
  private StatusSignal<Current> driveCurrentIn;
  private StatusSignal<Current> steerCurrentIn;

  protected boolean isCalibrating = false;

  public void configureModule(SwerveConfig config, SwerveModule.Corner corner) {
    this.constants = config;
    this.corner = corner;

    SwerveConfig.Module moduleConstants = getModuleConstants();

    driveMotor = new TalonFX(moduleConstants.driveMotorId(), config.canBus());

    TalonFXConfigurator driveConfig = driveMotor.getConfigurator();

    CTREUtils.check(driveConfig.apply(config.driveMotor().gains()));

    CTREUtils.check(
        driveConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)));

    CTREUtils.check(
        driveConfig.apply(
            new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(1)));

    CTREUtils.check(
        driveConfig.apply(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(config.driveMotor().maxCurrent())
                .withSupplyCurrentLimitEnable(true)));

    CTREUtils.check(
        driveConfig.apply(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(85)
                .withMotionMagicAcceleration(125)));

    steerEncoder = new CANcoder(moduleConstants.steerEncoderId(), config.canBus());

    CANcoderConfigurator steerEncoderConfig = steerEncoder.getConfigurator();

    CTREUtils.check(
        steerEncoderConfig.apply(
            new MagnetSensorConfigs()
                .withMagnetOffset(
                    moduleConstants.steerEncoderOffset().minus(corner.getModuleRotation()))));

    steerMotor = new TalonFX(moduleConstants.steerMotorId(), config.canBus());

    TalonFXConfigurator steerConfig = steerMotor.getConfigurator();

    CTREUtils.check(steerConfig.apply(new TalonFXConfiguration()));

    CTREUtils.check(steerConfig.apply(config.steerMotor().gains()));

    CTREUtils.check(
        steerConfig.apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)));

    CTREUtils.check(
        steerConfig.apply(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(config.steerMotor().maxCurrent())
                .withSupplyCurrentLimitEnable(true)));

    CTREUtils.check(
        steerConfig.apply(
            new FeedbackConfigs()
                .withFusedCANcoder(steerEncoder)
                .withRotorToSensorRatio(config.gearing().steer())));

    CTREUtils.check(
        steerConfig.apply(
            new MotionMagicConfigs()
                .withMotionMagicExpo_kV(2.7626 * 0.75)
                .withMotionMagicExpo_kA(0.3453 * 0.75)));

    CTREUtils.check(driveMotor.setPosition(Rotations.of(0)));

    setName(getModuleName(corner.index) + " Swerve Module");

    Logger.logSwerveModuleState(getName() + "/measuredState", this::getState);
    Logger.logSwerveModulePosition(getName() + "/measuredPosition", this::getPosition);
    Logger.logMeasure(getName() + "/consumedCurrent", this::getConsumedCurrent);
    Logger.logMeasure(getName() + "/driveWheelAngle", this::getDriveWheelAngle);
    Logger.logMeasure(getName() + "/driveWheelAngularVelocity", this::getDriveWheelAngularVelocity);
    Logger.logMeasure(getName() + "/driveRotorVelocity", () -> CTREUtils.unwrap(driveSpeedIn));

    StatusChecks.under(this).add("Drive Motor", driveMotor);
    StatusChecks.under(this).add("Steer Motor", steerMotor);
    StatusChecks.under(this).add("Steer Encoder", steerEncoder);

    setupStatusSignals();

    Logger.logNumber(
        getName() + "/steerVelocity",
        () -> CTREUtils.unwrap(steerMotor.getVelocity()).in(RotationsPerSecond));
  }

  /**
   * Gets the swerve drive configuration for this module.
   *
   * @return The {@link SwerveConfig} instance
   */
  public SwerveConfig getDrivetrainConstants() {
    return constants;
  }

  /**
   * Gets the swerve module configuration for this module.
   *
   * @return The {@link SwerveConfig.Module} instance
   */
  public SwerveConfig.Module getModuleConstants() {
    return constants.module(corner);
  }

  /**
   * Gets the corner of the robot that this module is located in.
   *
   * @return The {@link Corner} enum value
   */
  public Corner getModuleCorner() {
    return corner;
  }

  /**
   * Gets the drive motor for this module.
   *
   * @return The TalonFX drive motor controller
   */
  public TalonFX getDriveMotor() {
    return driveMotor;
  }

  /**
   * Gets the steer motor for this module.
   *
   * @return The TalonFX steer motor controller
   */
  public TalonFX getSteerMotor() {
    return steerMotor;
  }

  /**
   * Gets the steer encoder for this module.
   *
   * @return The steer CANcoder
   */
  public CANcoder getSteerEncoder() {
    return steerEncoder;
  }

  private void setupStatusSignals() {
    drivePositionIn = driveMotor.getPosition();
    steerAngleIn = steerMotor.getPosition();
    driveSpeedIn = driveMotor.getVelocity();
    steerVelocityIn = steerMotor.getVelocity();
    driveAccelerationIn = driveMotor.getAcceleration();
    steerAccelerationIn = steerMotor.getAcceleration();
    driveCurrentIn = driveMotor.getSupplyCurrent();
    steerCurrentIn = steerMotor.getSupplyCurrent();
  }

  private void refreshStatusSignals() {
    CTREUtils.check(
        BaseStatusSignal.refreshAll(
            drivePositionIn,
            steerAngleIn,
            driveSpeedIn,
            steerVelocityIn,
            driveAccelerationIn,
            steerAccelerationIn,
            driveCurrentIn,
            steerCurrentIn));
  }

  /**
   * Gets the current position of the drive and steer motors.
   *
   * @return The current {@link SwerveModulePosition}
   */
  public Distance getDrivePosition() {
    return constants.driveMotorRotorToMechanism(CTREUtils.unwrap(drivePositionIn));
  }

  /**
   * Gets the current angle of the drive wheel.
   *
   * @return The current {@link Angle} of the drive wheel.
   */
  public Angle getDriveWheelAngle() {
    return CTREUtils.unwrap(drivePositionIn).div(constants.gearing().drive());
  }

  /**
   * Gets the current linear velocity of the drive motor.
   *
   * @return The current {@link LinearVelocity}
   */
  public LinearVelocity getDriveVelocity() {
    return constants.driveMotorRotorToMechanism(CTREUtils.unwrap(driveSpeedIn));
  }

  /**
   * Gets the current angular velocity of the drive wheel.
   *
   * @return The current {@link AngularVelocity} of the drive wheel.
   */
  public AngularVelocity getDriveWheelAngularVelocity() {
    return CTREUtils.unwrap(driveSpeedIn).div(constants.gearing().drive());
  }

  /**
   * Gets the current linear acceleration of the drive motor.
   *
   * @return The current {@link LinearAcceleration}
   */
  public LinearAcceleration getDriveAcceleration() {
    return constants.driveMotorRotorToMechanism(CTREUtils.unwrap(driveAccelerationIn));
  }

  /**
   * Gets the current angular acceleration of the drive wheel.
   *
   * @return The current {@link AngularAcceleration} of the drive wheel.
   */
  public AngularAcceleration getDriveWheelAngularAcceleration() {
    return CTREUtils.unwrap(driveAccelerationIn).div(constants.gearing().drive());
  }

  /**
   * Gets the current angle of the steer motor.
   *
   * @return The current {@link Angle}
   */
  public Angle getSteerAngle() {
    return CTREUtils.unwrap(steerAngleIn);
  }

  /**
   * Gets the current velocity of the steer motor.
   *
   * @return The current {@link AngularVelocity}
   */
  public AngularVelocity getSteerVelocity() {
    return CTREUtils.unwrap(steerVelocityIn);
  }

  // Temporarily removed because getAcceleration() isn't working
  // TODO: Add after Phoenix 6 update
  // public AngularAcceleration getSteerAcceleration() {
  //   return steerAcceleration;
  // }

  /**
   * Gets the current current consumed by the module.
   *
   * @return The measured {@link Current}
   */
  public Current getConsumedCurrent() {
    return CTREUtils.unwrap(driveCurrentIn).plus(CTREUtils.unwrap(steerCurrentIn));
  }

  @Override
  public void periodic() {
    refreshStatusSignals();
  }

  public void drive(ControlRequest driveRequest, ControlRequest steerRequest) {
    if (isCalibrating) return;

    CTREUtils.check(driveMotor.setControl(driveRequest));
    CTREUtils.check(steerMotor.setControl(steerRequest));
  }

  /**
   * Gets the current state of the module.
   *
   * @return The measured {@link SwerveModuleState}
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
  }

  /**
   * Gets the current position of the module.
   *
   * @return The measured {@link SwerveModulePosition}
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerAngle()));
  }

  /**
   * Gets the transform of the module relative to the robot's center.
   *
   * @return The relative {@link Pose2d}
   */
  public Transform2d getRelativeTransform() {
    return new Transform2d(
        calculateRelativeTranslation(corner.index, constants.chassis()),
        new Rotation2d(getSteerAngle()));
  }

  /**
   * Creates a command to calibrate the steer motor.
   *
   * @param averageBusVoltage The expected average bus voltage to during calibration
   * @return The calibration {@link Command}
   */
  public Command calibrateSteerMotor() {
    return calibrateMotor(
        "steer",
        getSteerMotor(),
        log -> log.angularPosition(getSteerAngle()).angularVelocity(getSteerVelocity()));
  }

  /**
   * Creates a command to calibrate the drive motor.
   *
   * @param averageBusVoltage The expected average bus voltage to during calibration
   * @return The calibration {@link Command}
   */
  public Command calibrateDriveMotor() {
    return calibrateMotor(
        "drive",
        getDriveMotor(),
        log -> log.linearPosition(getDrivePosition()).linearVelocity(getDriveVelocity()));
  }

  private Command calibrateMotor(String motorName, TalonFX motor, Consumer<MotorLog> logEncoder) {
    SysIdRoutine calibrationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(7), Seconds.of(20)),
            new SysIdRoutine.Mechanism(
                voltage -> motor.setControl(new VoltageOut(voltage.in(Volts))),
                log ->
                    logEncoder.accept(
                        log.motor(
                                "swerve-module-"
                                    + SwerveModule.getModuleSysIdName(getModuleCorner().index)
                                    + "-"
                                    + motorName)
                            .voltage(CTREUtils.unwrap(motor.getMotorVoltage()))),
                this));

    return Commands.sequence(
        Commands.runOnce(() -> isCalibrating = true),
        Commands.waitSeconds(1.0),
        calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.runOnce(() -> motor.stopMotor()),
        Commands.waitSeconds(1.0),
        calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.runOnce(() -> motor.stopMotor()),
        Commands.waitSeconds(1.0),
        calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.runOnce(() -> motor.stopMotor()),
        Commands.waitSeconds(1.0),
        calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
        Commands.runOnce(() -> motor.stopMotor()),
        Commands.waitSeconds(1.0),
        Commands.runOnce(() -> isCalibrating = false));
  }

  @Override
  public void close() throws Exception {
    driveMotor.close();
    steerMotor.close();
    steerEncoder.close();
  }

  /**
   * Gets the name of a module given its index (e.g. 0 -> "Front Left").
   *
   * @param moduleIndex The index of the module
   * @return The name of the module, formatted with spaces and capitalization
   */
  public static String getModuleName(int moduleIndex) {
    return switch (moduleIndex) {
      case 0 -> "Front Left";
      case 1 -> "Front Right";
      case 2 -> "Back Left";
      case 3 -> "Back Right";
      default -> throw new IllegalArgumentException("Invalid module index");
    };
  }

  /**
   * Gets the name of a module given its index (e.g. 0 -> "front-left").
   *
   * @param moduleIndex The index of the module
   * @return The name of the module, formatted with hyphens and lowercase
   */
  public static String getModuleSysIdName(int moduleIndex) {
    return switch (moduleIndex) {
      case 0 -> "front-left";
      case 1 -> "front-right";
      case 2 -> "back-left";
      case 3 -> "back-right";
      default -> throw new IllegalArgumentException("Invalid module index");
    };
  }

  /** Represents a corner of the robot that a module can be on. */
  public static enum Corner {
    FRONT_LEFT(0, Rotations.of(0)),
    FRONT_RIGHT(1, Rotations.of(0.75)),
    BACK_LEFT(2, Rotations.of(0.25)),
    BACK_RIGHT(3, Rotations.of(0.5));

    public final int index;
    private final Angle moduleRotation;

    private Corner(int index, Angle moduleRotation) {
      this.index = index;
      this.moduleRotation = moduleRotation;
    }

    /**
     * Converts an index to a {@link Corner} object.
     *
     * @param index
     * @return
     */
    public static Corner fromIndex(int index) {
      return switch (index) {
        case 0 -> FRONT_LEFT;
        case 1 -> FRONT_RIGHT;
        case 2 -> BACK_LEFT;
        case 3 -> BACK_RIGHT;
        default -> throw new IllegalArgumentException("Invalid module index");
      };
    }

    public Angle getModuleRotation() {
      return moduleRotation;
    }
  }

  /**
   * Calculates the relative translation of a module given its corner index.
   *
   * @param cornerIndex The index of the corner
   * @param chassis The chassis configuration
   * @return The relative translation of the module
   */
  public static Translation2d calculateRelativeTranslation(
      int cornerIndex, SwerveConfig.Chassis chassis) {
    return new Translation2d(
        (cornerIndex < 2 ? 1. : -1.) * chassis.wheelBase().in(Meters) / 2.,
        (cornerIndex % 2 == 0 ? 1. : -1.) * chassis.trackWidth().in(Meters) / 2.);
  }

  public static SwerveModuleState optimizeStateForTalon(
      SwerveModuleState targetState, Angle currentAngle) {
    Angle difference = MeasureMath.minDifference(targetState.angle.getMeasure(), currentAngle);
    SwerveModuleState relativeOptimized =
        optimizeStateRelative(targetState.speedMetersPerSecond, difference);

    SwerveModuleState optimized =
        new SwerveModuleState(
            relativeOptimized.speedMetersPerSecond,
            new Rotation2d(currentAngle.plus(relativeOptimized.angle.getMeasure())));

    return optimized;
  }

  public static SwerveModuleState optimizeStateRelative(double speedMetersPerSecond, Angle angle) {
    if (angle.in(Rotations) < -0.25) {
      angle = angle.plus(Rotations.of(0.5));
      speedMetersPerSecond = -speedMetersPerSecond;
    } else if (angle.in(Rotations) > 0.25) {
      angle = angle.minus(Rotations.of(0.5));
      speedMetersPerSecond = -speedMetersPerSecond;
    }

    return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(angle));
  }
}
