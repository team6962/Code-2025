package com.team6962.lib.swerve.auto;

import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.KinematicsUtils;
import com.team6962.lib.utils.RotationUtils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.subsystems.vision.AprilTags;
import java.util.function.Supplier;

/**
 * {@code PoseEstimator} is a class that estimates the pose of a swerve drive robot using a
 * combination of odometry, gyroscope, and vision measurements. A PoseEstimator contains a {@link
 * SwerveDrivePoseEstimator} that is updated with the latest measurements. It also contains a {@link
 * SwerveGyroscope} that can be used to get and reset the robot's heading.
 *
 * <p>Vision data can be added with {@link #addVisionMeasurement(Pose2d, Time)}.
 */
public class PoseEstimator extends SubsystemBase {
  private SwerveDriveKinematics kinematics;
  private SwerveGyroscope gyroscope;
  private SwerveDrivePoseEstimator poseEstimator;

  private Supplier<SwerveModulePosition[]> modulePositionsSupplier;
  private Supplier<SwerveModuleState[]> moduleStatesSupplier;

  private SwerveModulePosition[] lastPositions;
  private SwerveModulePosition[] positionChanges = KinematicsUtils.blankModulePositions(4);
  private Twist2d chassisVelocity = new Twist2d();

  private Coordinates coordinates;

  public PoseEstimator(
      SwerveDriveKinematics kinematics,
      Supplier<SwerveModulePosition[]> modulePositions,
      Supplier<SwerveModuleState[]> moduleStates,
      Coordinates coordinates) {
    this.kinematics = kinematics;
    this.gyroscope = new SwerveGyroscope(() -> positionChanges, kinematics);
    this.modulePositionsSupplier = modulePositions;
    this.moduleStatesSupplier = moduleStates;
    this.coordinates = coordinates;

    lastPositions = modulePositions.get();

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            RotationUtils.fromAngle(gyroscope.getHeading()),
            modulePositions.get(),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    Logger.logSpeeds("Swerve Drive/Pose Estimator/estimatedSpeeds", this::getEstimatedSpeeds);
  }

  @Override
  public void periodic() {
    SwerveModulePosition[] modulePositions = this.modulePositionsSupplier.get();
    Time timestamp = Seconds.of(Timer.getFPGATimestamp());

    Logger.log("/PoseEstimator/gyroAngle", gyroscope.getHeading());
    Logger.log("/PoseEstimator/lastModulePositions", lastPositions);
    Logger.log("/PoseEstimator/modulePositions", modulePositions);
    Logger.log("/PoseEstimator/timestamp", timestamp);

    poseEstimator.updateWithTime(
        timestamp.in(Seconds), RotationUtils.fromAngle(gyroscope.getHeading()), modulePositions);
    AprilTags.injectVisionData(LIMELIGHT.APRILTAG_CAMERA_POSES, this);
    chassisVelocity =
        kinematics.toTwist2d(
            KinematicsUtils.toModulePositions(moduleStatesSupplier.get(), Seconds.of(1.0)));
    
    Logger.logObject("/PoseEstimator/chassisVelocity", chassisVelocity);

    positionChanges = KinematicsUtils.difference(modulePositions, lastPositions);
    Logger.log("/PoseEstimator/positionChanges", positionChanges);

    lastPositions = modulePositions;
  }

  public SwerveGyroscope getGyroscope() {
    return gyroscope;
  }

  public void addVisionMeasurement(Pose2d visionMeasurement, Time timestamp) {
    poseEstimator.addVisionMeasurement(coordinates.absoluteToAlliancePose(visionMeasurement), timestamp.in(Seconds));

    Logger.log("/PoseEstimator/lastVisionMeasurement/pose", visionMeasurement);
    Logger.log("/PoseEstimator/lastVisionMeasurement/timestamp", timestamp);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters, Time timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
      coordinates.absoluteToAlliancePose(visionRobotPoseMeters), timestamp.in(Seconds), visionMeasurementStdDevs);
    
    Logger.log("/PoseEstimator/lastVisionMeasurement/pose", visionRobotPoseMeters);
    Logger.log("/PoseEstimator/lastVisionMeasurement/timestamp", timestamp);
  }

  public void resetPosition(Pose2d expectedPose) {
    poseEstimator.resetPosition(
        RotationUtils.fromAngle(gyroscope.getHeading()),
        modulePositionsSupplier.get(),
        expectedPose);
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getFuturePose(Time time) {
    Pose2d currentPose = getEstimatedPose();

    if (chassisVelocity == null) return currentPose;

    double deltaSeconds = time.in(Seconds);
    Twist2d deltaTwist =
        new Twist2d(
            chassisVelocity.dx * deltaSeconds,
            chassisVelocity.dy * deltaSeconds,
            chassisVelocity.dtheta * deltaSeconds);

    return currentPose.exp(deltaTwist);
  }

  public Pose2d getEstimatedPose(Time timestamp) {
    return poseEstimator.sampleAt(timestamp.in(Seconds)).orElseGet(this::getEstimatedPose);
  }

  public ChassisSpeeds getEstimatedSpeeds() {
    return kinematics.toChassisSpeeds(moduleStatesSupplier.get());
  }
}
