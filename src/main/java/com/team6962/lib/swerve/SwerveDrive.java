package com.team6962.lib.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.team6962.lib.prepath.CustomPathfindingCommand;
import com.team6962.lib.swerve.auto.AutoBuilderWrapper;
import com.team6962.lib.swerve.auto.PathPrecomputing;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.CommandUtils;
import com.team6962.lib.utils.KinematicsUtils;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  private PathPrecomputing pathPrecomputing;

  /**
   * Create a new SwerveDrive object with the given configuration.
   *
   * @param constants The SwerveConfig object that contains the configuration for the swerve drive.
   */
  public SwerveDrive(SwerveConfig constants) {
    super(constants);

    setName("Swerve Drive");

    pathPrecomputing =
        new PathPrecomputing(getConstants().pathConstraints(), getConstants().pathRobotConfig());

    autoBuilder.configure(
        this::getEstimatedPose,
        this::resetPoseEstimate,
        () -> fieldToRobot(getEstimatedSpeeds()),
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
    return Commands.run(() -> moveRobotRelative(states.get()), useMotion());
  }

  public Command drive(SwerveModuleState[] states) {
    return driveModules(() -> states);
  }

  public Command driveSpeeds(Supplier<ChassisSpeeds> speeds) {
    return Commands.run(() -> moveFieldRelative(speeds.get()), useMotion());
  }

  public Command drive(ChassisSpeeds speeds) {
    return driveSpeeds(() -> speeds);
  }

  public Command driveTranslation(Supplier<Translation2d> translation) {
    return Commands.run(() -> moveFieldRelative(translation.get()), useTranslation());
  }

  public Command drive(Translation2d translation) {
    return driveTranslation(() -> translation);
  }

  public Command driveRotation(Supplier<Rotation2d> rotation) {
    return Commands.run(() -> moveRobotRelative(rotation.get()), useRotation());
  }

  public Command drive(Rotation2d rotation) {
    return driveRotation(() -> rotation);
  }

  public Command driveTwist(Twist2d twist) {
    return Commands.defer(() -> {
      SwerveModuleState[] relativeVelocities = getKinematics().toSwerveModuleStates(new ChassisSpeeds(twist.dx, twist.dy, twist.dtheta));

      double maxSpeedOfRelativeVelocities = 0.0;

      for (SwerveModuleState state : relativeVelocities) {
        maxSpeedOfRelativeVelocities = Math.max(maxSpeedOfRelativeVelocities, state.speedMetersPerSecond);
      }

      for (SwerveModuleState state : relativeVelocities) {
        state.speedMetersPerSecond /= maxSpeedOfRelativeVelocities;
      }

      return Commands.sequence(
        drive(KinematicsUtils.getStoppedStates(relativeVelocities))
            .until(() -> KinematicsUtils.isSimilarAngles(relativeVelocities, getModuleStates(), Degrees.of(6))),
        new Command() {
          private TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
              getMaxDriveSpeed().in(MetersPerSecond) * 0.25,
              getConstants().maxLinearAcceleration().in(MetersPerSecondPerSecond) * 0.25
            )
          );
          private double startTime = Timer.getFPGATimestamp();
    
          @Override
          public void initialize() {
            startTime = Timer.getFPGATimestamp();
          }
    
          @Override
          public void execute() {
            TrapezoidProfile.State currentState = profile.calculate(
              Timer.getFPGATimestamp() - startTime,
              new TrapezoidProfile.State(0, 0),
              new TrapezoidProfile.State(Math.hypot(twist.dx, twist.dy), 0)
            );
    
            SwerveModuleState[] targetStates = KinematicsUtils.multiply(relativeVelocities, currentState.velocity);
    
            moveRobotRelative(targetStates);
          }
          
          @Override
          public boolean isFinished() {
            return profile.isFinished(Timer.getFPGATimestamp() - startTime);
          }
        }
      );
    }, Set.of(useMotion()));
  }

  public Command driveTwistNotYet(Twist2d twist) {
    return Commands.defer(() -> {
      SwerveModuleState[] relativeVelocities = getKinematics().toSwerveModuleStates(new ChassisSpeeds(twist.dx, twist.dy, twist.dtheta));
      SwerveModulePosition[] targetPositions = KinematicsUtils.add(
        KinematicsUtils.toModulePositions(relativeVelocities, Seconds.of(1.0)),
        getModulePositions()
      );

      double maxSpeedOfRelativeVelocities = 0.0;

      for (SwerveModuleState state : relativeVelocities) {
        maxSpeedOfRelativeVelocities = Math.max(maxSpeedOfRelativeVelocities, state.speedMetersPerSecond);
      }

      for (SwerveModuleState state : relativeVelocities) {
        state.speedMetersPerSecond /= maxSpeedOfRelativeVelocities;
      }

      LinearVelocity[] maxLinearVelocities = new LinearVelocity[4];

      for (int i = 0; i < 4; i++) {
        maxLinearVelocities[i] = MetersPerSecond.of(relativeVelocities[i].speedMetersPerSecond * getConstants().maxDriveSpeed().in(MetersPerSecond));
      }

      LinearAcceleration[] maxLinearAccelerations = new LinearAcceleration[4];

      for (int i = 0; i < 4; i++) {
        maxLinearAccelerations[i] = MetersPerSecondPerSecond.of(relativeVelocities[i].speedMetersPerSecond * getConstants().maxLinearAcceleration().in(MetersPerSecondPerSecond));
      }

      ProfiledPositionMovement movement = new ProfiledPositionMovement(targetPositions, maxLinearVelocities, maxLinearAccelerations);

      return Commands.sequence(
        // drive(KinematicsUtils.getStoppedStates(relativeVelocities))
        //   .until(() -> KinematicsUtils.isSimilarAngles(relativeVelocities, getModuleStates(), Degrees.of(4))),
        Commands.run(() -> moveProfiledPositionMovement(movement))
      );
    }, Set.of(useTranslation(), useRotation()));
  }

  private class HeadingCommand extends Command {
    private PIDController pid;
    private Supplier<Rotation2d> heading;

    public HeadingCommand(Supplier<Rotation2d> heading) {
      this.heading = heading;

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
      Rotation2d targetHeading = heading.get();
      Rotation2d currentHeading = getEstimatedPose().getRotation();

      if (targetHeading == null) return;

      double output = pid.calculate(currentHeading.getRadians(), targetHeading.getRadians());

      output = MathUtil.clamp(output, -1, 1);

      moveRobotRelative(getAngularDriveVelocity(output));
    }

    @Override
    public void end(boolean interrupted) {
      pid.close();
    }
  }

  public Command driveHeading(Supplier<Rotation2d> heading) {
    return new HeadingCommand(heading);
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
    return Commands.defer(
        () -> {
          autoBuilder.setOutput(this::moveRobotRelative);

          return CommandUtils.withRequirements(
              AutoBuilder.pathfindToPose(target, getConstants().pathConstraints()), useMotion());
        },
        Set.of(useMotion()));
  }

  public Command pathfindTo(Pose2d target, LinearVelocity goalEndVelocity) {
    autoBuilder.setOutput(this::moveRobotRelative);

    return CommandUtils.withRequirements(
        AutoBuilder.pathfindToPose(target, getConstants().pathConstraints(), goalEndVelocity),
        useMotion());
  }

  public Command pathfindTo(Translation2d target) {
    autoBuilder.setOutput(this::moveRobotRelative);

    return CommandUtils.withRequirements(
        AutoBuilder.pathfindToPose(
            new Pose2d(target, Rotation2d.fromRotations(0)), getConstants().pathConstraints()),
        useMotion());
  }

  public Command pathfindTo(Translation2d target, LinearVelocity goalEndVelocity) {
    autoBuilder.setOutput(this::moveRobotRelative);

    return CommandUtils.withRequirements(
        AutoBuilder.pathfindToPose(
            new Pose2d(target, Rotation2d.fromRotations(0)),
            getConstants().pathConstraints(),
            goalEndVelocity),
        useMotion());
  }

  public AlignCommand alignTo(
      Supplier<Pose2d> target, Distance toleranceDistance, Angle toleranceAngle) {
    return new AlignCommand(target, toleranceDistance, toleranceAngle);
  }

  public AlignCommand alignTo(Pose2d target, Distance toleranceDistance, Angle toleranceAngle) {
    return alignTo(() -> target, toleranceDistance, toleranceAngle);
  }

  public AlignCommand alignTo(Supplier<Pose2d> target) {
    return alignTo(target, Inches.of(1.0), Degrees.of(4));
  }

  public AlignCommand alignTo(Pose2d target) {
    return alignTo(() -> target);
  }

  public boolean isWithinToleranceOf(
      Supplier<Pose2d> targetSupplier, Distance toleranceDistance, Angle toleranceAngle) {
    Pose2d target = targetSupplier.get();
    Pose2d current = getEstimatedPose();

    return current.getTranslation().getDistance(target.getTranslation())
            < toleranceDistance.in(Meters)
        && MeasureMath.minAbsDifference(
                current.getRotation().getMeasure(), target.getRotation().getMeasure())
            .lt(toleranceAngle);
  }

  public boolean isWithinToleranceOf(
      Pose2d target, Distance toleranceDistance, Angle toleranceAngle) {
    return isWithinToleranceOf(() -> target, toleranceDistance, toleranceAngle);
  }

  /**
   * A command to precisely align to a target position. This should not be used to drive to a target
   * far away, as it will not pathfind and may have unexpected behavior.
   */
  public class AlignCommand extends Command {
    private PIDController translationXPID;
    private PIDController translationYPID;
    private ProfiledPIDController rotationPID;
    private SimpleMotorFeedforward translateFeedforward;
    private SimpleMotorFeedforward rotateFeedforward;

    private Supplier<Pose2d> targetSupplier;
    private Distance toleranceDistance;
    private Angle toleranceAngle;
    private State state = State.TRANSLATING;
    private boolean endWithinTolerance = true;

    private enum State {
      TRANSLATING,
      ROTATING
    }

    public AlignCommand(
        Supplier<Pose2d> targetSupplier, Distance toleranceDistance, Angle toleranceAngle) {
      this.targetSupplier = targetSupplier;
      this.toleranceDistance = toleranceDistance;
      this.toleranceAngle = toleranceAngle;

      addRequirements(useMotion());
    }

    public AlignCommand withEndWithinTolerance(boolean value) {
      endWithinTolerance = value;

      return this;
    }

    @Override
    public void initialize() {
      translationXPID =
          new PIDController(
              getConstants().driveGains().fineTranslation().kP,
              getConstants().driveGains().fineTranslation().kI,
              getConstants().driveGains().fineTranslation().kD);
      translationYPID =
          new PIDController(
              getConstants().driveGains().fineTranslation().kP,
              getConstants().driveGains().fineTranslation().kI,
              getConstants().driveGains().fineTranslation().kD);
      rotationPID =
          new ProfiledPIDController(
              getConstants().driveGains().fineRotation().kP,
              getConstants().driveGains().fineRotation().kI,
              getConstants().driveGains().fineRotation().kD,
              new TrapezoidProfile.Constraints(
                  getConstants().maxRotationSpeed().in(RadiansPerSecond),
                  getConstants().maxAngularAcceleration().in(RadiansPerSecondPerSecond)));
      rotationPID.enableContinuousInput(-Math.PI, Math.PI);

      translateFeedforward = new SimpleMotorFeedforward(0.01, 0);
      rotateFeedforward = new SimpleMotorFeedforward(0.45, 0);
    }

    public Translation2d getTranslationError() {
      Pose2d target = targetSupplier.get();

      // Logger.log("/AlignCommand/estimatedTranslation", getEstimatedPose().getTranslation());
      // Logger.log("/AlignCommand/targetTranslation", target.getTranslation());

      Translation2d error = target.getTranslation().minus(getEstimatedPose().getTranslation());

      return error;
    }

    private Translation2d getTranslationOutput(Translation2d translationError) {
      // Logger.log("Swerve Drive/AlignCommand/translationError", translationError);

      double translationXOutput =
          translationXPID.calculate(-translationError.getX()); // Setpoint is 0, so need to invert
      double translationYOutput =
          translationYPID.calculate(-translationError.getY()); // Setpoint is 0, so need to invert

      translationXOutput +=
          translateFeedforward.calculateWithVelocities(
              getEstimatedSpeeds().vxMetersPerSecond, translationXOutput);
      translationYOutput +=
          translateFeedforward.calculateWithVelocities(
              getEstimatedSpeeds().vyMetersPerSecond, translationYOutput);

      if (getConstants().driveGains().maxAutonomousSpeed() != null) {
        double autoSpeed = getConstants().driveGains().maxAutonomousSpeed().in(MetersPerSecond);

        translationXOutput = MathUtil.clamp(translationXOutput, -autoSpeed, autoSpeed);
      }

      Translation2d translationOutput = new Translation2d(translationXOutput, translationYOutput);

      // Logger.log("Swerve Drive/AlignCommand/translationOutput", translationOutput);

      return translationOutput;
    }

    private Rotation2d getRotationError() {
      Pose2d target = targetSupplier.get();

      double rotationError =
          MeasureMath.minDifference(
                  getEstimatedPose().getRotation().getMeasure(), target.getRotation().getMeasure())
              .in(Radians);

      return Rotation2d.fromRadians(rotationError);
    }

    private Rotation2d getRotationOutput(Rotation2d rotationError) {
      // Logger.log("Swerve Drive/AlignCommand/rotationError", rotationError);

      double rotationOutput = rotationPID.calculate(rotationError.getRadians());

      rotationOutput +=
          rotateFeedforward.calculateWithVelocities(
              getEstimatedSpeeds().omegaRadiansPerSecond, rotationOutput);

      Rotation2d rotationOutputMeasure = Rotation2d.fromRadians(rotationOutput);

      // Logger.log("Swerve Drive/AlignCommand/rotationOutput", rotationOutputMeasure);

      return rotationOutputMeasure;
    }

    private void updateAdjustments(Translation2d translationError, Rotation2d rotationError) {
      boolean translationNeedsAdjustment =
          translationError.getNorm() > toleranceDistance.in(Meters);
      boolean rotationNeedsAdjustment =
          rotationError.getMeasure().abs(Rotations) > toleranceAngle.in(Rotations);
      double translationErrorRatio = translationError.getNorm() / toleranceDistance.in(Meters);
      double rotationErrorRatio =
          rotationError.getMeasure().abs(Rotation) / toleranceAngle.abs(Rotations);

      if (translationNeedsAdjustment && !rotationNeedsAdjustment) state = State.TRANSLATING;
      if (!translationNeedsAdjustment && rotationNeedsAdjustment) state = State.ROTATING;
      if (!translationNeedsAdjustment && !rotationNeedsAdjustment) {
        if (translationErrorRatio > rotationErrorRatio) state = State.TRANSLATING;
        else state = State.ROTATING;
      }

      Logger.log("Swerve Drive/AlignCommand/state", state.name());
      Logger.log("Swerve Drive/AlignCommand/needsTranslate", translationNeedsAdjustment);
      Logger.log("Swerve Drive/AlignCommand/needsRotate", rotationNeedsAdjustment);
    }

    @Override
    public void execute() {
      Translation2d translationError = getTranslationError();
      Rotation2d rotationError = getRotationError();

      updateAdjustments(translationError, rotationError);

      ChassisSpeeds speeds;

      if (state == State.TRANSLATING) {
        Translation2d translationOutput = new Translation2d(0, 0);
        translationOutput = getTranslationOutput(translationError);
        speeds = new ChassisSpeeds(translationOutput.getX(), translationOutput.getY(), 0);
      } else if (state == State.ROTATING) {
        Rotation2d rotationOutput = getRotationOutput(rotationError);
        speeds = new ChassisSpeeds(0, 0, rotationOutput.getRadians());
      } else {
        speeds = new ChassisSpeeds(0, 0, 0);
      }

      moveFieldRelative(speeds);
    }

    @Override
    public boolean isFinished() {
      if (!endWithinTolerance) return false;

      Pose2d target = targetSupplier.get();
      Pose2d current = getEstimatedPose();

      return current.getTranslation().getDistance(target.getTranslation())
              < toleranceDistance.in(Meters)
          && MeasureMath.minAbsDifference(
                  current.getRotation().getMeasure(), target.getRotation().getMeasure())
              .lt(toleranceAngle);
    }

    @Override
    public void end(boolean interrupted) {
      translationXPID.close();
      translationYPID.close();
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
  //           .linearVelocity(MetersPerSecond.of((leftMotor.getEncoder().getVelocity() +
  // rightMotor.getEncoder().getVelocity()) / 2)),
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

  public Command moveTowards(Translation2d target, LinearVelocity speed, Distance distance) {
    return Commands.defer(
        () -> {
          Translation2d offset = target.minus(getEstimatedPose().getTranslation());

          if (Math.abs(offset.getX()) <= 1e-6 && Math.abs(offset.getY()) <= 1e-6) {
            return Commands.waitTime(distance.div(speed));
          } else {
            return drive(offset.times(speed.in(MetersPerSecond) / offset.getNorm()))
                .withTimeout(distance.div(speed));
          }
        },
        Set.of(useTranslation()));
  }

  public Command driveTrapezoidalTo(
      Translation2d target,
      LinearVelocity maxVelocity,
      LinearAcceleration maxAcceleration,
      LinearVelocity targetEndVelocity,
      Distance positionTolerance,
      boolean timeLimit) {
    return new TrapezoidalTranslationCommand(
        () -> target,
        maxVelocity,
        maxAcceleration,
        targetEndVelocity,
        positionTolerance,
        timeLimit);
  }

  public TrapezoidalTranslationCommand driveTrapezoidalTo(Translation2d target) {
    SwerveConfig config = getConstants();

    return new TrapezoidalTranslationCommand(
        () -> target,
        config.maxDriveSpeed(),
        config.maxLinearAcceleration(),
        config.maxDriveSpeed(),
        Inches.of(4),
        true);
  }

  public class TrapezoidalTranslationCommand extends Command {
    private Supplier<Translation2d> targetSupplier;
    private Translation2d targetTranslation;

    public LinearVelocity maxVelocity;
    public LinearAcceleration maxAcceleration;

    public Distance positionTolerance;
    public LinearVelocity targetEndVelocity;

    public boolean timeLimit;

    private TrapezoidProfile profile;
    private Timer timer = new Timer();
    private Time endTime;

    public TrapezoidalTranslationCommand(
        Supplier<Translation2d> targetSupplier,
        LinearVelocity maxVelocity,
        LinearAcceleration maxAcceleration,
        LinearVelocity targetEndVelocity,
        Distance positionTolerance,
        boolean timeLimit) {
      this.targetSupplier = targetSupplier;
      this.maxVelocity = maxVelocity;
      this.maxAcceleration = maxAcceleration;
      this.positionTolerance = positionTolerance;
      this.targetEndVelocity = targetEndVelocity;
      this.timeLimit = timeLimit;

      addRequirements(useTranslation());
    }

    public TrapezoidalTranslationCommand withMaxVelocity(LinearVelocity maxVelocity) {
      this.maxVelocity = maxVelocity;
      return this;
    }

    public TrapezoidalTranslationCommand withMaxAcceleration(LinearAcceleration maxAcceleration) {
      this.maxAcceleration = maxAcceleration;
      return this;
    }

    public TrapezoidalTranslationCommand withPositionTolerance(Distance positionTolerance) {
      this.positionTolerance = positionTolerance;
      return this;
    }

    public TrapezoidalTranslationCommand withTargetEndVelocity(LinearVelocity targetEndVelocity) {
      this.targetEndVelocity = targetEndVelocity;
      return this;
    }

    public TrapezoidalTranslationCommand withTimeLimit(boolean timeLimit) {
      this.timeLimit = timeLimit;
      return this;
    }

    @Override
    public void initialize() {
      this.profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  maxVelocity.in(MetersPerSecond), maxAcceleration.in(MetersPerSecondPerSecond)));

      targetTranslation = targetSupplier.get();

      timer.restart();

      endTime = null;
    }

    @Override
    public void execute() {
      Translation2d currentTranslation = getEstimatedPose().getTranslation();
      Translation2d error = targetTranslation.minus(currentTranslation);
      Translation2d errorUnit = error.div(error.getNorm());

      double currentScalarDistance = error.getNorm();
      double targetScalarDistance = 0;

      Translation2d currentSpeeds = KinematicsUtils.getTranslation(getEstimatedSpeeds());
      double currentScalarSpeed =
          currentSpeeds.getX() * errorUnit.getX() + currentSpeeds.getY() * errorUnit.getY();

      TrapezoidProfile.State state =
          profile.calculate(
              timer.get(),
              new TrapezoidProfile.State(currentScalarDistance, -currentScalarSpeed),
              new TrapezoidProfile.State(
                  targetScalarDistance, -targetEndVelocity.in(MetersPerSecond)));

      if (timeLimit && endTime == null) {
        endTime = Seconds.of(profile.timeLeftUntil(0));
      }

      // Logger.log(
      //     "Swerve Drive/Trapezoidal Translation Command/currentDistance", currentScalarDistance);
      // Logger.log("Swerve Drive/Trapezoidal Translation Command/currentSpeed",
      // -currentScalarSpeed);
      // Logger.log(
      //     "Swerve Drive/Trapezoidal Translation Command/targetDistance", targetScalarDistance);
      // Logger.log(
      //     "Swerve Drive/Trapezoidal Translation Command/targetSpeed",
      //     -targetEndVelocity.in(MetersPerSecond));
      // Logger.log("Swerve Drive/Trapezoidal Translation Command/nextVelocity", -state.velocity);

      moveFieldRelative(errorUnit.times(-state.velocity));
    }

    @Override
    public boolean isFinished() {
      return ((endTime != null) && Seconds.of(timer.get()).gt(endTime))
          || getEstimatedPose().getTranslation().getDistance(targetTranslation)
              < positionTolerance.in(Meters);
    }
  }

  public Command pathfindBetweenWaypoints(
      Pose2d startPose,
      Pose2d endPose,
      Rotation2d startVelocityAngle,
      Rotation2d endVelocityAngle,
      LinearVelocity expectedStartVelocity,
      LinearVelocity maxEndVelocity) {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            List.of(
                new Pose2d(startPose.getTranslation(), startVelocityAngle),
                new Pose2d(endPose.getTranslation(), endVelocityAngle)));

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            getConstants().pathConstraints(),
            new IdealStartingState(expectedStartVelocity, startPose.getRotation()),
            new GoalEndState(maxEndVelocity.in(MetersPerSecond), endPose.getRotation()));

    path.getIdealTrajectory(getConstants().pathRobotConfig());

    return CommandUtils.withRequirements(
        new FollowPathCommand(
            path,
            this::getEstimatedPose,
            () -> fieldToRobot(getEstimatedSpeeds()),
            (speeds, ff) -> {
              moveRobotRelative(speeds);
            },
            getConstants().driveGains().pathController(),
            getConstants().pathRobotConfig(),
            () -> false),
        useMotion());
  }

  public Command pathfindToWaypoint(
      Pose2d precomputedStartPose,
      Pose2d endPose,
      Rotation2d precomputedStartVelocityAngle,
      Rotation2d endVelocityAngle,
      LinearVelocity precomputedStartVelocity,
      LinearVelocity maxEndVelocity) {
    Command pregenerated =
        pathfindBetweenWaypoints(
            precomputedStartPose,
            endPose,
            precomputedStartVelocityAngle,
            endVelocityAngle,
            precomputedStartVelocity,
            maxEndVelocity);

    return Commands.sequence(
        Commands.defer(
            () -> {
              Command attachment =
                  pathfindBetweenWaypoints(
                      getEstimatedPose(),
                      precomputedStartPose,
                      precomputedStartPose
                          .getTranslation()
                          .minus(getEstimatedPose().getTranslation())
                          .getAngle(),
                      precomputedStartVelocityAngle,
                      MetersPerSecond.of(
                          KinematicsUtils.getTranslation(getEstimatedSpeeds()).getNorm()),
                      precomputedStartVelocity);

              return attachment;
            },
            Set.of(useMotion())),
        pregenerated);
  }

  public Command pathfindToPrecomputed(Pose2d startPose, Pose2d endPose) {
    PathPrecomputing.Precompute precompute = pathPrecomputing.precompute(startPose, endPose);

    return Commands.defer(
        () -> {
          CustomPathfindingCommand pathfindToPose =
              new CustomPathfindingCommand(
                  endPose,
                  getConstants().pathConstraints(),
                  this::getEstimatedPose,
                  () -> fieldToRobot(getEstimatedSpeeds()),
                  (speeds, ff) -> moveRobotRelative(speeds),
                  getConstants().driveGains().pathController(),
                  getConstants().pathRobotConfig(),
                  useMotion());

          if (precompute.isAvailable() && precompute.isStartPoseNear(getEstimatedPose())) {
            System.out.println("Using precomputed path!");
            pathfindToPose.currentPath = precompute.getPath();
            pathfindToPose.currentTrajectory = precompute.getTrajectory();
            pathfindToPose.timer.restart();

            List<Waypoint> waypoints = precompute.getPath().getWaypoints();

            for (int i = 0; i < waypoints.size(); i++) {
              System.out.println(" (waypoint " + i + ") " + waypoints.get(i).anchor());
            }

            System.out.println(" (start) " + precompute.getStartPose());
            System.out.println(" (end) " + precompute.getEndPose());
            System.out.println(" (current) " + getEstimatedPose());
          } else {
            System.out.println("Using non-precomputed path.");

            System.out.println(" (current) " + getEstimatedPose());
            System.out.println(" (end) " + endPose);
          }

          return pathfindToPose;

          // PathPlannerPath precomputedPath = precompute.getPath();

          // if (precomputedPath != null && precompute.isStartPoseNear(getEstimatedPose())) {
          //   return CommandUtils.withRequirements(new FollowPathCommand(
          //     precomputedPath,
          //     this::getEstimatedPose,
          //     () -> fieldToRobot(getEstimatedSpeeds()),
          //     (speeds, ff) -> {
          //       moveRobotRelative(speeds);
          //     },
          //     getConstants().driveGains().pathController(),
          //     getConstants().pathRobotConfig(),
          //     () -> false
          //   ), useMotion());
          // } else {
          //   return pathfindTo(endPose);
          // }
        },
        Set.of(useMotion()));
  }

  public Command calibrateWheelSize() {
    return Commands.defer(() -> new Command() {
      Angle initialGyroAngle;
      Angle[] initialWheelAngles = new Angle[4];

      @Override
      public void initialize() {
        initialGyroAngle = getContinuousGyroscopeAngle();

        for (int i = 0; i < 4; i++) {
          initialWheelAngles[i] = getModules()[i].getDriveWheelAngle();
        }
      }

      @Override
      public void end(boolean interrupted) {
        Angle gyroAngleChange = Rotations.of(getContinuousGyroscopeAngle().minus(initialGyroAngle).in(Rotations));
        Distance driveRadius = getConstants().chassis().driveRadius();

        Logger.log("Swerve Drive/Wheel Size Calibration/Gyro Angle Change", gyroAngleChange);

        Angle[] wheelAngleChanges = new Angle[4];

        for (int i = 0; i < 4; i++) {
          wheelAngleChanges[i] = Rotations.of(getModules()[i].getDriveWheelAngle().minus(initialWheelAngles[i]).abs(Rotations));

          Logger.log("Swerve Drive/Wheel Size Calibration/Wheel Angle Change " + SwerveModule.getModuleName(i), wheelAngleChanges[i]);
        }

        Distance[] wheelDiameters = new Distance[4];

        for (int i = 0; i < 4; i++) {
          wheelDiameters[i] = driveRadius.times(gyroAngleChange.div(wheelAngleChanges[i])).times(2);

          Logger.log("Swerve Drive/Wheel Size Calibration/Diameter " + SwerveModule.getModuleName(i), wheelDiameters[i]);
        }

        Distance averageDiameter = Meters.of(0);

        for (Distance diameter : wheelDiameters) {
          averageDiameter = averageDiameter.plus(diameter);
        }

        averageDiameter = averageDiameter.div(4);

        Logger.log("Swerve Drive/Wheel Size Calibration/Average Diameter", averageDiameter);
      }
    }, Set.of()).alongWith(Commands.sequence(
      Commands.waitSeconds(1),
      drive(Rotation2d.fromDegrees(getConstants().maxRotationSpeed().div(8).in(DegreesPerSecond))).withTimeout(10),
      Commands.waitSeconds(1)
    ));
  }
}
