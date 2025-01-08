package com.team6962.lib.swerve.auto;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import com.team6962.lib.utils.KinematicsUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * An interface that provides methods for converting between different coordinate
 * systems. It should be implemented by the SwerveDrive class to allow for easy
 * conversion between robot, absolute, and alliance coordinates.
 */
public interface Coordinates {
    /**
     * The width of the field (from one end of the barge to the other).
     */
    public static final Distance FIELD_WIDTH = Inches.of(317.330000);

    /**
     * The length of the field (from the alliance wall to the opposite alliance
     * wall).
     */
    public static final Distance FIELD_LENGTH = Inches.of(690.875958);

    /**
     * The center of the field, as a Translation2d.
     */
    public static final Translation2d FIELD_CENTER = new Translation2d(FIELD_LENGTH.div(2), FIELD_WIDTH.div(2));

    public static enum LocalSystem {
        ROBOT, ABSOLUTE, ALLIANCE
    }

    public static enum FieldSystem {
        ABSOLUTE, ALLIANCE
    }

    public Pose2d getEstimatedPose();

    public static Optional<Boolean> isAllianceInverted() {
        return DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red);
    }

    public static boolean knowsAlliance() {
        return DriverStation.getAlliance().isPresent();
    }

    public default ChassisSpeeds robotToAbsoluteSpeeds(ChassisSpeeds speeds) {
        return KinematicsUtils.rotateSpeeds(speeds, getEstimatedPose().getRotation().unaryMinus());
    }

    public default ChassisSpeeds absoluteToRobotSpeeds(ChassisSpeeds speeds) {
        return KinematicsUtils.rotateSpeeds(speeds, getEstimatedPose().getRotation());
    }

    public default ChassisSpeeds allianceToAbsoluteSpeeds(ChassisSpeeds speeds) {
        return isAllianceInverted().orElse(false) ?
            KinematicsUtils.rotateSpeeds(speeds, Rotation2d.fromRotations(0.5)) :
            speeds;
    }

    public default ChassisSpeeds absoluteToAllianceSpeeds(ChassisSpeeds speeds) {
        return allianceToAbsoluteSpeeds(speeds);
    }

    public default ChassisSpeeds robotToAllianceSpeeds(ChassisSpeeds speeds) {
        return absoluteToAllianceSpeeds(robotToAbsoluteSpeeds(speeds));
    }

    public default ChassisSpeeds allianceToRobotSpeeds(ChassisSpeeds speeds) {
        return absoluteToRobotSpeeds(allianceToAbsoluteSpeeds(speeds));
    }

    public default ChassisSpeeds convertSpeeds(ChassisSpeeds speeds, LocalSystem from, LocalSystem to) {
        ChassisSpeeds absolute = switch (from) {
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

    public default Translation2d robotToAbsoluteVelocity(Translation2d velocity) {
        return velocity.rotateBy(getEstimatedPose().getRotation().unaryMinus());
    }

    public default Translation2d absoluteToRobotVelocity(Translation2d velocity) {
        return velocity.rotateBy(getEstimatedPose().getRotation());
    }

    public default Translation2d allianceToAbsoluteVelocity(Translation2d velocity) {
        return isAllianceInverted().orElse(false) ?
            velocity.rotateBy(Rotation2d.fromRotations(0.5)) :
            velocity;
    }

    public default Translation2d absoluteToAllianceVelocity(Translation2d velocity) {
        return allianceToAbsoluteVelocity(velocity);
    }

    public default Translation2d robotToAllianceVelocity(Translation2d velocity) {
        return absoluteToAllianceVelocity(robotToAbsoluteVelocity(velocity));
    }

    public default Translation2d allianceToRobotVelocity(Translation2d velocity) {
        return absoluteToRobotVelocity(allianceToAbsoluteVelocity(velocity));
    }

    public default Translation2d convertVelocity(Translation2d velocity, LocalSystem from, LocalSystem to) {
        Translation2d absolute = switch (from) {
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

    public default Rotation2d robotToAbsoluteAngle(Rotation2d angle) {
        return angle.rotateBy(getEstimatedPose().getRotation().unaryMinus());
    }

    public default Rotation2d absoluteToRobotAngle(Rotation2d angle) {
        return angle.rotateBy(getEstimatedPose().getRotation());
    }

    public default Rotation2d allianceToAbsoluteAngle(Rotation2d angle) {
        return isAllianceInverted().orElse(false) ?
            angle.rotateBy(Rotation2d.fromRotations(0.5)) :
            angle;
    }

    public default Rotation2d absoluteToAllianceAngle(Rotation2d angle) {
        return allianceToAbsoluteAngle(angle);
    }

    public default Rotation2d robotToAllianceAngle(Rotation2d angle) {
        return absoluteToAllianceAngle(robotToAbsoluteAngle(angle));
    }

    public default Rotation2d allianceToRobotAngle(Rotation2d angle) {
        return absoluteToRobotAngle(allianceToAbsoluteAngle(angle));
    }

    public default Rotation2d convertAngle(Rotation2d angle, LocalSystem from, LocalSystem to) {
        Rotation2d absolute = switch (from) {
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

    public default Pose2d allianceToAbsolutePose(Pose2d pose) {
        if (isAllianceInverted().orElse(false)) {
            return pose.transformBy(new Transform2d(FIELD_CENTER.times(-1), Rotation2d.fromDegrees(0)))
                .rotateBy(Rotation2d.fromRotations(0.5))
                .transformBy(new Transform2d(FIELD_CENTER, Rotation2d.fromDegrees(0)));
        } else {
            return pose;
        }
    }

    // TODO: Finish implementing pose coordinate conversions for FieldSystems
}
