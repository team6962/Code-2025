package com.team6962.lib.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.team6962.lib.swerve.auto.AutoBuilderWrapper;
import com.team6962.lib.swerve.auto.Coordinates;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.KinematicsUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    return Commands.run(
        () -> setMovement(convertAngle(rotation.get(), system, Coordinates.MovementSystem.ROBOT)),
        useRotation());
  }

  public Command driveRotation(Supplier<Rotation2d> rotation) {
    return driveRotation(rotation, Coordinates.MovementSystem.ALLIANCE);
  }

  public Command drive(Rotation2d rotation) {
    return driveRotation(() -> rotation);
  }

  public Command driveHeading(Supplier<Rotation2d> heading, Coordinates.MovementSystem system) {
    Command command =
        new Command() {
          private PIDController pid;

          @Override
          public void initialize() {
            PIDConstants constants = getConstants().driveGains().rotation();
            pid = new PIDController(constants.kP, constants.kI, constants.kD);

            pid.enableContinuousInput(-Math.PI, Math.PI);
          }

          @Override
          public void execute() {
            Rotation2d error = heading.get().minus(getEstimatedPose().getRotation());
            double output = pid.calculate(error.getRadians());

            setMovement(
                convertAngle(new Rotation2d(output), system, Coordinates.MovementSystem.ROBOT));
          }

          @Override
          public void end(boolean interrupted) {
            pid.close();
          }
        };

    command.addRequirements(useRotation());

    return command;
  }

  public Command driveHeading(Supplier<Rotation2d> heading) {
    return driveHeading(heading, Coordinates.MovementSystem.ALLIANCE);
  }

  public Command driveHeading(Rotation2d heading) {
    return driveHeading(() -> heading);
  }

    public Command facePointCommand(Supplier<Translation2d> point, Rotation2d rotationOffset) {
        return Commands.run(
            () -> facePoint(point.get(), rotationOffset)
        );
    }

    public void facePoint(Translation2d point, Rotation2d rotationOffset) {
        double time = 0.02;

        if (point == null) {
            // TODO: add and velocity
            driveHeading(getEstimatedPose().getRotation());
            return;
        }

        if (point.getDistance(getEstimatedPose().getTranslation()) < 1.0 && RobotState.isAutonomous()) {
            return;
        }


        Translation2d currentPosition = getEstimatedPose().getTranslation();
        Translation2d futurePosition = getEstimatedPose().getTranslation().plus(KinematicsUtils.getTranslation(getEstimatedSpeeds()).times(time));
        
        Rotation2d currentTargetHeading = point.minus(currentPosition).getAngle().plus(rotationOffset);
        Rotation2d futureTargetHeading = point.minus(futurePosition).getAngle().plus(rotationOffset);
        
        double addedVelocity = futureTargetHeading.minus(currentTargetHeading).getRadians() / time;
        if (getEstimatedPose().getTranslation().getDistance(point) < 1.0) {
            addedVelocity = 0.0;
        }


        driveHeading(currentTargetHeading);
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

  // public Command pathfindTo(
  //     Pose2d target,
  //     boolean ignoreRotation,
  //     GoalEndState endState,
  //     Distance rotationDelayDistance) {
  //   return Commands.defer(
  //       () -> {
  //         Pose2d startPose =
  //             new Pose2d(
  //                 getEstimatedPose().getTranslation(),
  //                 KinematicsUtils.getAngle(
  //                     getEstimatedSpeeds().vxMetersPerSecond,
  //                     getEstimatedSpeeds().vyMetersPerSecond));

  //         List<Waypoint> bezierPoints =
  //             PathPlannerPath.waypointsFromPoses(
  //                 ignoreRotation
  //                     ? new Pose2d(getEstimatedPose().getTranslation(), new Rotation2d())
  //                     : startPose,
  //                 ignoreRotation ? new Pose2d(target.getTranslation(), new Rotation2d()) :
  // target);

  //         return pathfindThrough(
  //             bezierPoints, ignoreRotation, endState, rotationDelayDistance.in(Meters));
  //       },
  //       Set.of());
  // }

  // public Command pathfindThroughPoints(List<Translation2d> points) {
  //   return pathfindThroughPoints(points, new GoalEndState(0, new Rotation2d()));
  // }

  // public Command pathfindThroughPoints(List<Translation2d> points, GoalEndState endState) {
  //   return Commands.defer(
  //       () -> {
  //         List<Waypoint> bezierPoints =
  //             PathPlannerPath.waypointsFromPoses(
  //                 points.stream().map(p -> new Pose2d(p, new Rotation2d())).toList());

  //         return pathfindThrough(bezierPoints, false, endState, 0);
  //       },
  //       Set.of());
  // }

  // public Command pathfindThroughPoses(List<Pose2d> poses) {
  //   return pathfindThroughPoses(poses, new GoalEndState(0, new Rotation2d()));
  // }

  // public Command pathfindThroughPoses(List<Pose2d> poses, GoalEndState endState) {
  //   return pathfindThroughPoses(poses, endState, 0);
  // }

  // public Command pathfindThroughPoses(
  //     List<Pose2d> poses, GoalEndState endState, double rotationDelayDistance) {
  //   return Commands.defer(
  //       () -> {
  //         List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(poses);

  //         return pathfindThrough(bezierPoints, false, endState, rotationDelayDistance);
  //       },
  //       Set.of());
  // }

  // public Command pathfindThrough(
  //     List<Waypoint> bezierPoints,
  //     boolean ignoreRotation,
  //     GoalEndState endState,
  //     double rotationDelayDistance) {
  //   return Commands.defer(() -> {
  //       autoBuilder.setOutput((speeds) -> {
  //           if (ignoreRotation)
  //               setMovement(new Translation2d(speeds.vxMetersPerSecond,
  // speeds.vyMetersPerSecond));
  //           else setMovement(speeds);
  //       });

  //       PathPlannerPath goalPath = new PathPlannerPath(
  //           bezierPoints,
  //           getConstants().pathConstraints(),
  //           getPathplannerStartingState(),
  //           endState
  //       );

  //       return AutoBuilder.pathfindThenFollowPath(goalPath, getConstants().pathConstraints());

  //   //     return new FollowPathCommand(
  //   //         path,
  //   //         this::getEstimatedPose,
  //   //         getPoseEstimator()::getEstimatedSpeeds,
  //   //         (speeds, feedforwards) -> {
  //   //             if (ignoreRotation)
  //   //                 setMovement(new Translation2d(speeds.vxMetersPerSecond,
  // speeds.vyMetersPerSecond));
  //   //             else setMovement(speeds);
  //   //         },
  //   //         getConstants().driveGains().pathController(),
  //   //         getConstants().pathRobotConfig(),
  //   //         () -> false);
  //   }, ignoreRotation ? Set.of(useTranslation()) : Set.of(useMotion()));
  // }

  // private IdealStartingState getPathplannerStartingState() {
  //   return new IdealStartingState(
  //       Math.hypot(getEstimatedSpeeds().vxMetersPerSecond,
  // getEstimatedSpeeds().vyMetersPerSecond),
  //       Rotation2d.fromRadians(getEstimatedSpeeds().omegaRadiansPerSecond));
  // }

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
}
