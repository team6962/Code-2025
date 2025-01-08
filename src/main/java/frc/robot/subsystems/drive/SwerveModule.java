// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.Constants.Constants.SWERVE_DRIVE.DRIVE_MOTOR_PROFILE;
import frc.robot.Constants.Constants.SWERVE_DRIVE.MODULE_CONFIG;
import frc.robot.Constants.Constants.SWERVE_DRIVE.PHYSICS;
import frc.robot.Constants.Constants.SWERVE_DRIVE.STEER_MOTOR_PROFILE;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.software.MathUtils.SwerveMath;

public class SwerveModule extends SubsystemBase {
  private SparkMax driveMotor, steerMotor;
  private RelativeEncoder driveEncoder, steerEncoder;
  private CANcoder absoluteSteerEncoder;
  private SparkClosedLoopController drivePID, steerPID;
  private SwerveModuleState targetState = new SwerveModuleState();
  private String name;
  private int corner;
  private Rotation2d absoluteSteerDirection = new Rotation2d();
  private double driveVelocity = 0.0;
  private double drivePosition = 0.0;
  private Rotation2d relativeSteerDirection = new Rotation2d();

  private boolean isCalibrating = false;
  
  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(
    DRIVE_MOTOR_PROFILE.kS,
    DRIVE_MOTOR_PROFILE.kV,
    DRIVE_MOTOR_PROFILE.kA
  );

  public SwerveModule(MODULE_CONFIG config, int corner, String name) {
    this.corner = corner;
    this.name = name;

    driveMotor           = new SparkMax(config.CAN_DRIVE(), MotorType.kBrushless);
    steerMotor           = new SparkMax(config.CAN_STEER(), MotorType.kBrushless);
    absoluteSteerEncoder = new CANcoder(config.CAN_ENCODER());
    steerEncoder         = steerMotor.getEncoder();
    driveEncoder         = driveMotor.getEncoder();
    drivePID             = driveMotor.getClosedLoopController();
    steerPID             = steerMotor.getClosedLoopController();

    double encoderOffset = config.ENCODER_OFFSET();
    switch (corner) {
      case 0:
        encoderOffset += 0.0;
        break;
      case 1:
        encoderOffset += 0.25;
        break;
      case 2:
        encoderOffset += -0.25;
        break;
      case 3:
        encoderOffset += 0.5;
        break;
      default:
    }

    encoderOffset %= 2;
    encoderOffset = (encoderOffset > 1.0) ? encoderOffset - 2.0 : (encoderOffset < -1.0) ? encoderOffset + 2.0 : encoderOffset;

    MagnetSensorConfigs magConfig = new MagnetSensorConfigs();
    // magConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
    magConfig.withMagnetOffset(encoderOffset);
    BaseStatusSignal.setUpdateFrequencyForAll(50, absoluteSteerEncoder.getAbsolutePosition(), absoluteSteerEncoder.getFaultField(), absoluteSteerEncoder.getVersion());
    absoluteSteerEncoder.optimizeBusUtilization();

    SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
    SparkMaxConfig steerMotorConfig = new SparkMaxConfig();

    SparkMaxUtil.configureAndLog(this, driveMotor, driveMotorConfig, false, IdleMode.kBrake, PHYSICS.SLIPLESS_CURRENT_LIMIT, PHYSICS.SLIPLESS_CURRENT_LIMIT);
    SparkMaxUtil.configureAndLog(this, steerMotor, steerMotorConfig, true, IdleMode.kCoast);
    SparkMaxUtil.configureEncoder(driveMotorConfig, SWERVE_DRIVE.DRIVE_ENCODER_CONVERSION_FACTOR);
    SparkMaxUtil.configureEncoder(steerMotorConfig, SWERVE_DRIVE.STEER_ENCODER_CONVERSION_FACTOR);
    SparkMaxUtil.configurePID(this, driveMotorConfig, DRIVE_MOTOR_PROFILE.kP, DRIVE_MOTOR_PROFILE.kI, DRIVE_MOTOR_PROFILE.kD, 0.0, false);
    SparkMaxUtil.configurePID(this, steerMotorConfig, STEER_MOTOR_PROFILE.kP, STEER_MOTOR_PROFILE.kI, STEER_MOTOR_PROFILE.kD, 0.0, true);
    
    // driveMotor.setClosedLoopRampRate(SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY / SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION);
    // driveMotor.setOpenLoopRampRate(SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY / SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION);
    
    SparkMaxUtil.save(driveMotor, driveMotorConfig);
    SparkMaxUtil.save(steerMotor, steerMotorConfig);
    
    SparkMaxUtil.configureCANStatusFrames(driveMotor, true, true);
    SparkMaxUtil.configureCANStatusFrames(steerMotor, false, true);

    seedSteerEncoder();

    String logPath = "module" + name + "/";
    Logger.logNumber(logPath + "relativeSteerDirection", () -> relativeSteerDirection.getDegrees());
    Logger.logNumber(logPath + "absoluteSteerDirection", () -> absoluteSteerDirection.getDegrees());

    StatusChecks.under(this).add(name + "canCoderHasFaults", () -> absoluteSteerEncoder.getFaultField().getValue() == 0);
    StatusChecks.under(this).add(name + "canCoderIsConnected", () -> absoluteSteerEncoder.getVersion().getValue() != 0);
  }


  public void periodic() {
    relativeSteerDirection = Rotation2d.fromRadians(steerEncoder.getPosition());
    absoluteSteerDirection = new Rotation2d(absoluteSteerEncoder.getAbsolutePosition().getValue());
    driveVelocity = driveEncoder.getVelocity();
    drivePosition = driveEncoder.getPosition();

    if (!ENABLED_SYSTEMS.ENABLE_DRIVE) return;
    if (isCalibrating) return;

    drive(targetState);

    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.SWERVE_DRIVE) stop();

  }
  
  public void drive(SwerveModuleState state) {
    double speedMetersPerSecond = state.speedMetersPerSecond;
    double radians = state.angle.getRadians();
    
    if (SWERVE_DRIVE.DO_ANGLE_ERROR_SPEED_REDUCTION) {
      speedMetersPerSecond *= Math.cos(SwerveMath.angleDistance(radians, getMeasuredState().angle.getRadians()));
    }
    
    drivePID.setReference(
      speedMetersPerSecond,
      ControlType.kVelocity,
      ClosedLoopSlot.kSlot0,
      driveFF.calculate(speedMetersPerSecond)
    );

    steerPID.setReference(
      radians,
      ControlType.kPosition
    );

    if (state.speedMetersPerSecond == 0 && Math.abs(getRelativeSteerDirection().minus(getAbsoluteSteerDirection()).getDegrees()) > 0.5) {
      seedSteerEncoder();
    }
  }
  
  public void setTargetState(SwerveModuleState state) {
    targetState = SwerveModuleState.optimize(state, getMeasuredState().angle);
  }
  
  public void stop() {
    targetState = new SwerveModuleState(0.0, getMeasuredState().angle);
    // steerMotor.stopMotor();
    // driveMotor.stopMotor();
  }
  
  /**
   * Seeds the position of the built-in relative encoder with the absolute position of the steer CANCoder.
   * This is because the CANCoder polls at a lower rate than we'd like, so we essentially turn the relative encoder into an fast-updating absolute encoder.
   * Also the built-in SparkMaxPIDControllers require a compatible encoder to run the faster 1kHz closed loop 
   */
  public void seedSteerEncoder() {
    steerEncoder.setPosition(getAbsoluteSteerDirection().getRadians());
  }

  public Rotation2d getRelativeSteerDirection() {
    return relativeSteerDirection;
  }
  
  private Rotation2d getAbsoluteSteerDirection() {
    return absoluteSteerDirection;
  }

  public SwerveModuleState getTargetState() {
    return targetState;
  }
  
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(driveVelocity, getAbsoluteSteerDirection());
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(drivePosition, getMeasuredState().angle);
  }

  public static double calcWheelVelocity(double power) {
    return power * Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY;
  }

  public double getTotalCurrent() {
    return driveMotor.getOutputCurrent() + steerMotor.getOutputCurrent();
  }
  
  public Pose2d getPose(Pose2d robotPose) {
    Pose2d relativePose = new Pose2d();
    if (corner == 0) relativePose = new Pose2d(
      SWERVE_DRIVE.WHEELBASE / 2.0,
      SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (corner == 1) relativePose = new Pose2d(
      SWERVE_DRIVE.WHEELBASE / 2.0,
      -SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (corner == 2) relativePose = new Pose2d(
      -SWERVE_DRIVE.WHEELBASE / 2.0,
      SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (corner == 3) relativePose = new Pose2d(
      -SWERVE_DRIVE.WHEELBASE / 2.0,
      -SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    return relativePose.relativeTo(new Pose2d(
      new Translation2d(),
      robotPose.getRotation().times(-1.0)
    )).relativeTo( new Pose2d(
      -robotPose.getX(),
      -robotPose.getY(),
      new Rotation2d()
    ));
  }

  public Command calibrateSteerMotor() {
    SysIdRoutine calibrationRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Voltage volts) -> {
          steerMotor.setVoltage(volts.in(Volts));
        },
        log -> {
          log.motor("module-steer-" + name)
              .voltage(Volts.of(steerMotor.getBusVoltage() * steerMotor.getAppliedOutput()))
              .angularPosition(Radians.of(steerEncoder.getPosition()))
              .angularVelocity(RadiansPerSecond.of(steerEncoder.getVelocity()));
        },
        this
      )
    );

    return Commands.sequence(
      Commands.runOnce(() -> doCalibrationPrep()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> steerMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> steerMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> steerMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> steerMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> undoCalibrationPrep())
    );
  }

  public Command calibrateDriveMotor() {
    SysIdRoutine calibrationRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Voltage volts) -> {
          driveMotor.setVoltage(volts.in(Volts));
        },
        log -> {
          log.motor("module-drive-" + name)
              .voltage(Volts.of(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput()))
              .linearPosition(Meters.of(driveEncoder.getPosition()))
              .linearVelocity(MetersPerSecond.of(driveEncoder.getVelocity()));
        },
        this
      )
    );

    return Commands.sequence(
      Commands.runOnce(() -> doCalibrationPrep()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> driveMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> driveMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> driveMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> driveMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> undoCalibrationPrep())
    );
  }

  private void doCalibrationPrep() {
    isCalibrating = true;
    SparkMaxUtil.configureCANStatusFrames(steerMotor, true, true);
  }

  private void undoCalibrationPrep() {
    isCalibrating = false;
    SparkMaxUtil.configureCANStatusFrames(steerMotor, false, false);
  }
}