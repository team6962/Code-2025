package com.team6962.lib.swerve.auto;

import static edu.wpi.first.units.Units.Inches;

import com.team6962.lib.utils.KinematicsUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

/** An interface that provides methods for converting between different coordinate systems. */
public interface Coordinates {
  /** The width of the field (from one end of the barge to the other). */
  public static final Distance FIELD_WIDTH = Inches.of(317.330000);

  /** The length of the field (from the alliance wall to the opposite alliance wall). */
  public static final Distance FIELD_LENGTH = Inches.of(690.875958);

  /** The center of the field, as a Translation2d. */
  public static final Translation2d FIELD_CENTER =
      new Translation2d(FIELD_LENGTH.div(2), FIELD_WIDTH.div(2));

  /** An enum that represents the different coordinate systems that can be used for movements. */
  public static enum MovementSystem {
    ROBOT,
    ABSOLUTE,
    ALLIANCE
  }

  /** An enum that represents the different coordinate systems that can be used for poses. */
  public static enum PoseSystem {
    ABSOLUTE,
    ALLIANCE
  }

  /** Gets the estimated pose of the robot. */
  public Pose2d getEstimatedPose();

  /**
   * Gets whether alliance-relative coordinates should be inverted.
   *
   * <p>This method will return an empty {@link Optional} if the alliance has not yet been received
   * from the Driver Station.
   */
  public static Optional<Boolean> isAllianceInverted() {
    return DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red);
  }

  /** Returns whether the alliance has been received from the Driver Station. */
  public static boolean knowsAlliance() {
    return DriverStation.getAlliance().isPresent();
  }

  public default Rotation2d getAbsoluteEstimatedHeading() {
    return isAllianceInverted().orElse(false)
        ? getEstimatedPose().getRotation().rotateBy(Rotation2d.fromRotations(0.5))
        : getEstimatedPose().getRotation();
  }

  /** Converts {@link ChassisSpeeds} from robot coordinates to absolute coordinates. */
  public default ChassisSpeeds robotToAbsoluteSpeeds(ChassisSpeeds speeds) {
    return ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getEstimatedPose().getRotation());
  }

  /** Converts {@link ChassisSpeeds} from absolute coordinates to robot coordinates. */
  public default ChassisSpeeds absoluteToRobotSpeeds(ChassisSpeeds speeds) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getEstimatedPose().getRotation());
  }

  /** Converts {@link ChassisSpeeds} from alliance coordinates to absolute coordinates. */
  public default ChassisSpeeds allianceToAbsoluteSpeeds(ChassisSpeeds speeds) {
    return isAllianceInverted().orElse(false)
        ? KinematicsUtils.allianceInvertSpeeds(speeds)
        : speeds;
  }

  /** Converts {@link ChassisSpeeds} from absolute coordinates to alliance coordinates. */
  public default ChassisSpeeds absoluteToAllianceSpeeds(ChassisSpeeds speeds) {
    return allianceToAbsoluteSpeeds(speeds);
  }

  /** Converts {@link ChassisSpeeds} from robot coordinates to alliance coordinates. */
  public default ChassisSpeeds robotToAllianceSpeeds(ChassisSpeeds speeds) {
    return ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getAbsoluteEstimatedHeading());
  }

  /** Converts {@link ChassisSpeeds} from alliance coordinates to robot coordinates. */
  public default ChassisSpeeds allianceToRobotSpeeds(ChassisSpeeds speeds) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAbsoluteEstimatedHeading());
  }

  /** Converts {@link ChassisSpeeds} from one coordinate system to another. */
  public default ChassisSpeeds convertSpeeds(
      ChassisSpeeds speeds, MovementSystem from, MovementSystem to) {
    ChassisSpeeds absolute =
        switch (from) {
          case ROBOT -> robotToAbsoluteSpeeds(speeds);
          case ABSOLUTE -> speeds;
          case ALLIANCE -> allianceToAbsoluteSpeeds(speeds);
        };

    return switch (to) {
      case ROBOT -> absoluteToRobotSpeeds(absolute);
      case ABSOLUTE -> absolute;
      case ALLIANCE -> absoluteToAllianceSpeeds(absolute);
    };
  }

  /** Converts a {@link Translation2d} from robot coordinates to absolute coordinates. */
  public default Translation2d robotToAbsoluteVelocity(Translation2d velocity) {
    return velocity.rotateBy(getEstimatedPose().getRotation());
  }

  /** Converts a {@link Translation2d} from absolute coordinates to robot coordinates. */
  public default Translation2d absoluteToRobotVelocity(Translation2d velocity) {
    return velocity.rotateBy(getEstimatedPose().getRotation().unaryMinus());
  }

  /** Converts a {@link Translation2d} from alliance coordinates to absolute coordinates. */
  public default Translation2d allianceToAbsoluteVelocity(Translation2d velocity) {
    return isAllianceInverted().orElse(false)
        ? velocity.rotateBy(Rotation2d.fromRotations(0.5))
        : velocity;
  }

  /** Converts a {@link Translation2d} from absolute coordinates to alliance coordinates. */
  public default Translation2d absoluteToAllianceVelocity(Translation2d velocity) {
    return allianceToAbsoluteVelocity(velocity);
  }

  /** Converts a {@link Translation2d} from robot coordinates to alliance coordinates. */
  public default Translation2d robotToAllianceVelocity(Translation2d velocity) {
    return absoluteToAllianceVelocity(robotToAbsoluteVelocity(velocity));
  }

  /** Converts a {@link Translation2d} from alliance coordinates to robot coordinates. */
  public default Translation2d allianceToRobotVelocity(Translation2d velocity) {
    return absoluteToRobotVelocity(allianceToAbsoluteVelocity(velocity));
  }

  /** Converts a {@link Translation2d} from one coordinate system to another. */
  public default Translation2d convertVelocity(
      Translation2d velocity, MovementSystem from, MovementSystem to) {
    Translation2d absolute =
        switch (from) {
          case ROBOT -> robotToAbsoluteVelocity(velocity);
          case ABSOLUTE -> velocity;
          case ALLIANCE -> allianceToAbsoluteVelocity(velocity);
        };

    return switch (to) {
      case ROBOT -> absoluteToRobotVelocity(absolute);
      case ABSOLUTE -> absolute;
      case ALLIANCE -> absoluteToAllianceVelocity(absolute);
    };
  }

  /** Converts a {@link Rotation2d} from robot coordinates to absolute coordinates. */
  public default Rotation2d robotToAbsoluteAngle(Rotation2d angle) {
    return angle.rotateBy(getEstimatedPose().getRotation().unaryMinus());
  }

  /** Converts a {@link Rotation2d} from absolute coordinates to robot coordinates. */
  public default Rotation2d absoluteToRobotAngle(Rotation2d angle) {
    return angle.rotateBy(getEstimatedPose().getRotation());
  }

  /** Converts a {@link Rotation2d} from alliance coordinates to absolute coordinates. */
  public default Rotation2d allianceToAbsoluteAngle(Rotation2d angle) {
    return isAllianceInverted().orElse(false)
        ? angle.rotateBy(Rotation2d.fromRotations(0.5))
        : angle;
  }

  /** Converts a {@link Rotation2d} from absolute coordinates to alliance coordinates. */
  public default Rotation2d absoluteToAllianceAngle(Rotation2d angle) {
    return allianceToAbsoluteAngle(angle);
  }

  /** Converts a {@link Rotation2d} from robot coordinates to alliance coordinates. */
  public default Rotation2d robotToAllianceAngle(Rotation2d angle) {
    return absoluteToAllianceAngle(robotToAbsoluteAngle(angle));
  }

  /** Converts a {@link Rotation2d} from alliance coordinates to robot coordinates. */
  public default Rotation2d allianceToRobotAngle(Rotation2d angle) {
    return absoluteToRobotAngle(allianceToAbsoluteAngle(angle));
  }

  /** Converts a {@link Rotation2d} from one coordinate system to another. */
  public default Rotation2d convertAngle(Rotation2d angle, MovementSystem from, MovementSystem to) {
    Rotation2d absolute =
        switch (from) {
          case ROBOT -> robotToAbsoluteAngle(angle);
          case ABSOLUTE -> angle;
          case ALLIANCE -> allianceToAbsoluteAngle(angle);
        };

    return switch (to) {
      case ROBOT -> absoluteToRobotAngle(absolute);
      case ABSOLUTE -> absolute;
      case ALLIANCE -> absoluteToAllianceAngle(absolute);
    };
  }

  /** Converts a {@link Pose2d} from alliance coordinates to absolute coordinates. */
  public default Pose2d allianceToAbsolutePose(Pose2d pose) {
    if (isAllianceInverted().orElse(false)) {
      return pose.transformBy(new Transform2d(FIELD_CENTER.times(-1), Rotation2d.fromDegrees(0)))
          .rotateBy(Rotation2d.fromRotations(0.5))
          .transformBy(new Transform2d(FIELD_CENTER, Rotation2d.fromDegrees(0)));
    } else {
      return pose;
    }
  }

  /** Converts a {@link Pose2d} from absolute coordinates to alliance coordinates. */
  public default Pose2d absoluteToAlliancePose(Pose2d pose) {
    return allianceToAbsolutePose(pose);
  }

  /** Converts a {@link Pose2d} from one coordinate system to another. */
  public default Pose2d convertPose(Pose2d pose, PoseSystem from, PoseSystem to) {
    Pose2d absolute =
        switch (from) {
          case ABSOLUTE -> pose;
          case ALLIANCE -> allianceToAbsolutePose(pose);
        };

    return switch (to) {
      case ABSOLUTE -> absolute;
      case ALLIANCE -> absoluteToAlliancePose(absolute);
    };
  }
}
