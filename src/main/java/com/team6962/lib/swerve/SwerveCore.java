package com.team6962.lib.swerve;

import com.team6962.lib.swerve.auto.PoseEstimator;
import com.team6962.lib.swerve.auto.RobotCoordinates;
import com.team6962.lib.swerve.module.SimulatedModule;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.KinematicsUtils;
import com.team6962.lib.utils.MeasureMath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;

/**
 * The "core" of the swerve drive system. This class is responsible for managing the swerve modules,
 * the pose estimator, and the current movement the drivetrain is performing. {@code SwerveCore} is
 * extended by {@link SwerveDrive} to provide the {@link Command}-based API for controlling the
 * drivetrain.
 */
public class SwerveCore extends SubsystemBase implements RobotCoordinates {
  private PoseEstimator poseEstimator;
  private SwerveModule[] modules;
  private SwerveDriveKinematics kinematics;
  private SwerveConfig constants;

  private SwerveMovement currentMovement;
  private LinearVelocity maxSpeed;

  public SwerveCore(SwerveConfig constants) {
    this.constants = constants;

    modules = new SwerveModule[4];

    for (int i = 0; i < 4; i++) {
      SwerveModule module = RobotBase.isReal() ? new SwerveModule() : new SimulatedModule();
      module.configureModule(constants, SwerveModule.Corner.fromIndex(i));

      modules[i] = module;
    }

    kinematics = KinematicsUtils.kinematicsFromChassis(constants.chassis());
    poseEstimator =
        new PoseEstimator(kinematics, () -> getModulePositions(), () -> getModuleStates());

    currentMovement = new SwerveMovement(kinematics);

    Logger.logPose("Swerve Drive/robotPose", poseEstimator::getEstimatedPose);

    maxSpeed = constants.maxDriveSpeed();
  }

  /**
   * Sets the maximum speed of the drive motors. This should only be called inside commands that
   * require the max speed subsystem.
   *
   * @param maxSpeed the maximum speed to set, values higher than {@code
   *     getConstants().maxDriveSpeed()} are ignored.
   */
  public void setMaxDriveSpeed(LinearVelocity maxSpeed) {
    this.maxSpeed = maxSpeed;
  }

  public SwerveConfig getConstants() {
    return constants;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(modules)
        .map(SwerveModule::getPosition)
        .toArray(SwerveModulePosition[]::new);
  }

  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
  }

  public SwerveModule[] getModules() {
    return modules;
  }

  public Pose2d[] getModulePoses() {
    Pose2d[] poses = new Pose2d[4];

    for (int i = 0; i < 4; i++) {
      Transform2d relativeTransform = modules[i].getRelativeTransform();

      poses[i] = poseEstimator.getEstimatedPose().transformBy(relativeTransform);
    }

    return poses;
  }

  @Override
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPose();
  }

  public Pose2d getEstimatedPose(Time timestamp) {
    return poseEstimator.getEstimatedPose(timestamp);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters, Time timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestamp, visionMeasurementStdDevs);
  }

  public void addVisionHeading(Rotation2d visionHeading) {
    poseEstimator.addVisionHeading(visionHeading);
  }

  public void resetPoseEstimate(Pose2d expectedPose) {
    poseEstimator.resetPoseEstimate(expectedPose);
  }

  public void resetHeadingEstimate(Rotation2d expectedHeading) {
    poseEstimator.resetHeadingEstimate(expectedHeading);
  }

  public Pose2d getFuturePose(Time time) {
    return poseEstimator.getFuturePose(time);
  }

  public ChassisSpeeds getEstimatedSpeeds() {
    return poseEstimator.getEstimatedSpeeds();
  }

  /** Should be called after CommandScheduler.run() to mimimize latency. */
  public void latePeriodic() {
    SwerveModuleState[] states = currentMovement.getStates();

    if (states == null) {
      states = KinematicsUtils.getStoppedStates(getModuleStates());
    }

    states =
        KinematicsUtils.desaturateWheelSpeeds(
            states, MeasureMath.min(maxSpeed, constants.maxDriveSpeed()));

    Logger.log(getName() + "/targetModuleStates", states);

    Pose2d[] poses = getModulePoses();

    for (int i = 0; i < 4; i++) {
      modules[i].driveState(new SwerveModuleState(states[i].speedMetersPerSecond, states[i].angle));
      poses[i] =
          new Pose2d(
              poses[i].getTranslation(),
              poseEstimator.getEstimatedPose().getRotation().plus(states[i].angle));
    }

    Logger.getField().getObject("Target Module Poses").setPoses(poses);

    currentMovement.clear();
  }

  public void moveRobotRelative(SwerveModuleState[] states) {
    currentMovement.setStates(states);
  }

  public void moveRobotRelative(ChassisSpeeds speeds) {
    currentMovement.setSpeeds(speeds);
  }

  public void moveRobotRelative(Translation2d translation) {
    currentMovement.setTranslation(translation);
  }

  public void moveRobotRelative(Rotation2d rotation) {
    currentMovement.setRotation(rotation);
  }

  public void moveRobotRelative(AngularVelocity rotation) {
    currentMovement.setRotation(MeasureMath.fromMeasure(rotation));
  }

  public void moveFieldRelative(ChassisSpeeds speeds) {
    moveRobotRelative(fieldToRobot(speeds));
  }

  public void moveFieldRelative(Translation2d translation) {
    moveRobotRelative(fieldToRobot(translation));
  }
}
