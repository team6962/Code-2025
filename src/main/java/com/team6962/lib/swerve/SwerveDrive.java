package com.team6962.lib.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.team6962.lib.swerve.auto.AutoBuilderWrapper;
import com.team6962.lib.swerve.auto.Coordinates;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.KinematicsUtils;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.Supplier;

/**
 * The main class for the swerve drive system. This class extends {@link SwerveCore} to provide the
 * {@link Command}-based API for controlling the drivetrain.
 */
public class SwerveDrive extends SwerveCore {
  /** Subsystem for translation commands to require. */
  private SubsystemBase translationSubsystem = new SubsystemBase() {};

  /** Subsystem for rotation commands to require. */
  private SubsystemBase rotationSubsysem = new SubsystemBase() {};

  /** Subsystem for max speed commands to require. */
  private SubsystemBase maxSpeedSubsystem = new SubsystemBase() {};

  private AutoBuilderWrapper autoBuilder = new AutoBuilderWrapper();

  /**
   * Create a new SwerveDrive object with the given configuration.
   *
   * @param constants The SwerveConfig object that contains the configuration for the swerve drive.
   */
  public SwerveDrive(SwerveConfig constants) {
    super(constants);

    setName("Swerve Drive");

    autoBuilder.configure(
        this::getEstimatedPose,
        this.getPoseEstimator()::resetPosition,
        this.getPoseEstimator()::getEstimatedSpeeds,
        getConstants().driveGains().pathController(),
        getConstants().pathRobotConfig(),
        () -> false);

    maxSpeedSubsystem.setDefaultCommand(limitSpeed(getConstants().maxDriveSpeed()));
  }

  @Override
  public void periodic() {
    Field2d field = Logger.getField();
    // Logger.log("Swerve Drive/AlgaePosition", );
    field.setRobotPose(getEstimatedPose());

    FieldObject2d modules = field.getObject("Swerve Modules");

    modules.setPoses(getModulePoses());
  }

  public Pose2d getFuturePose(Time time) {
    return getPoseEstimator().getFuturePose(time);
  }

  /**
   * Returns the estimated chassis speeds of the robot in the given coordinate system.
   *
   * @return The estimated speeds as a ChassisSpeeds object
   */
  public ChassisSpeeds getEstimatedSpeeds(Coordinates.MovementSystem system) {
    return convertSpeeds(
        getPoseEstimator().getEstimatedSpeeds(), Coordinates.MovementSystem.ROBOT, system);
  }

  /**
   * Returns the estimated alliance-relative chassis speeds of the robot.
   *
   * @return The estimated speeds as a ChassisSpeeds object
   */
  public ChassisSpeeds getEstimatedSpeeds() {
    return getEstimatedSpeeds(Coordinates.MovementSystem.ALLIANCE);
  }

  public SubsystemBase useTranslation() {
    return translationSubsystem;
  }

  public SubsystemBase useRotation() {
    return rotationSubsysem;
  }

  public SubsystemBase[] useMotion() {
    return new SubsystemBase[] {translationSubsystem, rotationSubsysem};
  }

  public SubsystemBase useMaxSpeed() {
    return maxSpeedSubsystem;
  }

  public Command limitSpeed(Supplier<LinearVelocity> maxSpeed) {
    return Commands.run(() -> setMaxDriveSpeed(maxSpeed.get()), useMaxSpeed());
  }

  public Command limitSpeed(LinearVelocity maxSpeed) {
    return limitSpeed(() -> maxSpeed);
  }

  public LinearVelocity getLinearDriveVelocity(double powerFraction) {
    return getConstants().maxDriveSpeed().times(powerFraction);
  }

  public AngularVelocity getAngularDriveVelocity(double powerFraction) {
    return getConstants().maxRotationSpeed().times(powerFraction);
  }

  public Command driveModules(Supplier<SwerveModuleState[]> states) {
    return Commands.run(() -> setMovement(states.get()), useMotion());
  }

  public Command drive(SwerveModuleState[] states) {
    return driveModules(() -> states);
  }

  public Command driveSpeeds(Supplier<ChassisSpeeds> speeds, Coordinates.MovementSystem system) {
    Logger.log("Swerve Drive/driveSpeeds", speeds.get());

    return Commands.run(
        () -> setMovement(convertSpeeds(speeds.get(), system, Coordinates.MovementSystem.ROBOT)),
        useMotion());
  }

  public Command driveSpeeds(Supplier<ChassisSpeeds> speeds) {
    return driveSpeeds(speeds, Coordinates.MovementSystem.ALLIANCE);
  }

  public Command drive(ChassisSpeeds speeds) {
    return driveSpeeds(() -> speeds);
  }

  public Command driveTranslation(
      Supplier<Translation2d> translation, Coordinates.MovementSystem system) {
    return Commands.run(
        () -> {
          setMovement(convertVelocity(translation.get(), system, Coordinates.MovementSystem.ROBOT));
        },
        useTranslation());
  }

  public Command driveTranslation(Supplier<Translation2d> translation) {
    return driveTranslation(translation, Coordinates.MovementSystem.ALLIANCE);
  }

  public Command drive(Translation2d translation) {
    return driveTranslation(() -> translation);
  }

  public Command driveRotation(Supplier<Rotation2d> rotation, Coordinates.MovementSystem system) {
    return Commands.run(() -> setMovement(rotation.get()), useRotation());
  }

  public Command driveRotation(Supplier<Rotation2d> rotation) {
    return driveRotation(rotation, Coordinates.MovementSystem.ALLIANCE);
  }

  public Command drive(Rotation2d rotation) {
    return driveRotation(() -> rotation);
  }

  private class HeadingCommand extends Command {
    private PIDController pid;
    private Supplier<Rotation2d> heading;
    private Coordinates.PoseSystem system;

    public HeadingCommand(Supplier<Rotation2d> heading, Coordinates.PoseSystem system) {
      this.heading = heading;
      this.system = system;

      addRequirements(useRotation());
    }

    @Override
    public void initialize() {
      PIDConstants constants = getConstants().driveGains().rotation();
      pid = new PIDController(constants.kP, constants.kI, constants.kD);

      pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
      Rotation2d targetHeading = convertAngle(heading.get(), system, Coordinates.PoseSystem.ALLIANCE);
      Rotation2d currentHeading = getEstimatedPose().getRotation();

      if (targetHeading == null) return;

      double output = pid.calculate(currentHeading.getRadians(), targetHeading.getRadians());

      output = MathUtil.clamp(output, -1, 1);

      setMovement(MeasureMath.fromMeasure(getAngularDriveVelocity(output)));
    }

    @Override
    public void end(boolean interrupted) {
      pid.close();
    }
  }

  public Command driveHeading(Supplier<Rotation2d> heading, Coordinates.PoseSystem system) {
    return new HeadingCommand(heading, system);
  }

  public Command driveHeading(Supplier<Rotation2d> heading) {
    return driveHeading(heading, Coordinates.PoseSystem.ALLIANCE);
  }

  public Command driveHeading(Rotation2d heading) {
    return driveHeading(() -> heading);
  }

  public Command facePoint(Supplier<Translation2d> point, Supplier<Rotation2d> offset) {
    return driveHeading(
        () -> {
          Translation2d currentPoint = point.get();

          if (currentPoint == null) return null;

          Translation2d currentPoseTranslation = getEstimatedPose().getTranslation();
          Translation2d translationDistance = currentPoint.minus(currentPoseTranslation);
          Rotation2d targetHeading = translationDistance.getAngle().plus(offset.get());

          return targetHeading;
        });
  }

  public Command facePoint(Supplier<Translation2d> point) {
    return facePoint(point, () -> Rotation2d.fromRotations(0));
  }

  public Command facePoint(Translation2d point) {
    return facePoint(() -> point);
  }

  public Command stop() {
    return driveModules(() -> KinematicsUtils.getStoppedStates(getModuleStates()));
  }

  public Command park() {
    return driveModules(() -> KinematicsUtils.getParkedStates());
  }

  public Command pathfindTo(Pose2d target) {
    autoBuilder.setOutput(speeds -> setMovement(speeds));

    return AutoBuilder.pathfindToPose(target, getConstants().pathConstraints());
  }

  public Command pathfindTo(Pose2d target, LinearVelocity goalEndVelocity) {
    autoBuilder.setOutput(speeds -> setMovement(speeds));

    return AutoBuilder.pathfindToPose(target, getConstants().pathConstraints(), goalEndVelocity);
  }

  public Command pathfindTo(Translation2d target) {
    autoBuilder.setOutput(speeds -> setMovement(KinematicsUtils.getTranslation(speeds)));

    return AutoBuilder.pathfindToPose(
        new Pose2d(target, Rotation2d.fromRotations(0)), getConstants().pathConstraints());
  }

  public Command pathfindTo(Translation2d target, LinearVelocity goalEndVelocity) {
    autoBuilder.setOutput(speeds -> setMovement(KinematicsUtils.getTranslation(speeds)));

    return AutoBuilder.pathfindToPose(
        new Pose2d(target, Rotation2d.fromRotations(0)),
        getConstants().pathConstraints(),
        goalEndVelocity);
  }

  public Command alignTo(
      Supplier<Pose2d> target, Distance toleranceDistance, Angle toleranceAngle) {
    return new AlignCommand(target, toleranceDistance, toleranceAngle);
  }

  public Command alignTo(Pose2d target, Distance toleranceDistance, Angle toleranceAngle) {
    return alignTo(() -> target, toleranceDistance, toleranceAngle);
  }

  public Command alignTo(Supplier<Pose2d> target) {
    return alignTo(target, Inches.of(1), Degrees.of(4));
  }

  public Command alignTo(Pose2d target) {
    return alignTo(() -> target);
  }

  private class AlignCommand extends Command {
    private PIDController translationXPID;
    private PIDController translationYPID;
    private PIDController rotationPID;

    private Supplier<Pose2d> targetSupplier;
    private Distance toleranceDistance;
    private Angle toleranceAngle;

    public AlignCommand(
        Supplier<Pose2d> targetSupplier, Distance toleranceDistance, Angle toleranceAngle) {
      this.targetSupplier = targetSupplier;
      this.toleranceDistance = toleranceDistance;
      this.toleranceAngle = toleranceAngle;

      translationXPID =
          new PIDController(
              getConstants().driveGains().translation().kP,
              getConstants().driveGains().translation().kI,
              getConstants().driveGains().translation().kD);
      translationYPID =
          new PIDController(
              getConstants().driveGains().translation().kP,
              getConstants().driveGains().translation().kI,
              getConstants().driveGains().translation().kD);
      rotationPID =
          new PIDController(
              getConstants().driveGains().rotation().kP,
              getConstants().driveGains().rotation().kI,
              getConstants().driveGains().rotation().kD);
      rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
      Pose2d target = targetSupplier.get();

      double translationXError =
          target.getTranslation().getX() - getEstimatedPose().getTranslation().getX();
      double translationYError =
          target.getTranslation().getY() - getEstimatedPose().getTranslation().getY();
      double rotationError =
          target.getRotation().getRadians() - getEstimatedPose().getRotation().getRadians();

      double translationXOutput = translationXPID.calculate(translationXError);
      double translationYOutput = translationYPID.calculate(translationYError);
      double rotationOutput = rotationPID.calculate(rotationError);

      setMovement(
          allianceToRobotSpeeds(
              new ChassisSpeeds(
                  translationXOutput * 0.5, translationYOutput * 0.5, rotationOutput * 0.5)));
    }

    @Override
    public boolean isFinished() {
      Pose2d target = targetSupplier.get();
      Pose2d current = getEstimatedPose();

      return current.getTranslation().getDistance(target.getTranslation())
              < toleranceDistance.in(Meters)
          && MeasureMath.differenceUnderHalf(
                  current.getRotation().getMeasure(), target.getRotation().getMeasure())
              .lt(toleranceAngle);
    }
  }

  public Command cancelDrive() {
    return Commands.run(
        () -> {
          if (translationSubsystem.getCurrentCommand() != null) {
            translationSubsystem.getCurrentCommand().cancel();
          }

          if (rotationSubsysem.getCurrentCommand() != null) {
            rotationSubsysem.getCurrentCommand().cancel();
          }
        },
        useMotion());
  }

  // public Command calibrate() {
  //   SysIdRoutine calibrationRoutine = new SysIdRoutine(
  //     new SysIdRoutine.Config(Volts.per(Second).of(4.0), Volts.of(2.0), Seconds.of(3.0)),
  //     new SysIdRoutine.Mechanism(
  //       voltage -> drive(new ChassisSpeeds(voltage.in(Volts) / 12.0 * , 0, 0)),
  //       log -> log.motor("elevator")
  //           .voltage(Volts.of((
  //             leftMotor.getAppliedOutput() * leftMotor.getBusVoltage() +
  //             rightMotor.getAppliedOutput() * rightMotor.getBusVoltage()) / 2.0))
  //           .linearPosition(getAverageHeight())
  //           .linearVelocity(MetersPerSecond.of((leftMotor.getEncoder().getVelocity() + rightMotor.getEncoder().getVelocity()) / 2)),
  //       this));

  //   return Commands.sequence(
  //     Commands.waitSeconds(1.0),
  //     calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
  //     Commands.runOnce(() -> { leftMotor.stopMotor(); rightMotor.stopMotor(); }),
  //     Commands.waitSeconds(1.0),
  //     calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
  //     Commands.runOnce(() -> { leftMotor.stopMotor(); rightMotor.stopMotor(); }),
  //     Commands.waitSeconds(1.0),
  //     calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
  //     Commands.runOnce(() -> { leftMotor.stopMotor(); rightMotor.stopMotor(); }),
  //     Commands.waitSeconds(1.0),
  //     calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
  //     Commands.runOnce(() -> { leftMotor.stopMotor(); rightMotor.stopMotor(); }),
  //     Commands.waitSeconds(1.0)
  //   );
  // }
}
