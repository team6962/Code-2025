package frc.robot.Constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.Constants.SWERVE;

public final class ReefPositioning {
    private ReefPositioning() {}

    public static final Translation2d REEF_CENTER = new Translation2d(
        Units.inchesToMeters(176.745545),
        Units.inchesToMeters(158.499093)
    );

    public static final Distance REEF_TO_EDGE = Inches.of(32.745545);
    // public static final Distance REEF_TO_POLE = Inches.of(30.738196);
    public static final Distance BETWEEN_POLES = Inches.of(12.937756);
    public static final Distance ROBOT_TO_EDGE_PLACE = SWERVE.CONFIG.chassis().outerLength().div(2);
    public static final Distance ROBOT_TO_EDGE_ALIGN = ROBOT_TO_EDGE_PLACE.plus(Inches.of(8));

    public static final Translation2d PLACE_RELATIVE = new Translation2d(
        REEF_TO_EDGE.plus(ROBOT_TO_EDGE_PLACE).in(Meters),
        BETWEEN_POLES.div(2).in(Meters)
    );

    public static final Translation2d ALIGN_RELATIVE = new Translation2d(
        REEF_TO_EDGE.plus(ROBOT_TO_EDGE_ALIGN).in(Meters),
        BETWEEN_POLES.div(2).in(Meters)
    );

    private static Pose2d relativeToFieldPose(Pose2d relativePose, Rotation2d rotation) {
        Translation2d reefTranslation = relativePose.getTranslation().rotateBy(rotation);
        Translation2d fieldTranslation = REEF_CENTER.plus(reefTranslation);

        return new Pose2d(fieldTranslation, rotation);
    }

    private static Pose2d reflectPose(Pose2d pose) {
        return new Pose2d(new Translation2d(pose.getX(), -pose.getY()), pose.getRotation());
    }

    private static Pose2d relativeToFieldAndReflect(Translation2d relativeTranslation, Rotation2d rotation, boolean reverse) {
        Pose2d relativePose = new Pose2d(relativeTranslation, new Rotation2d());

        if (reverse) {
            relativePose = reflectPose(relativePose);
        }

        return relativeToFieldPose(relativePose, rotation);
    }

    // Poles are CCW starting at right (WPILib convention)
    private static Rotation2d getRotationOfPole(int face) {
        return Rotation2d.fromDegrees(60. * Math.round(face / 2.));
    }

    private static boolean getMirroringOfPole(int pole) {
        return pole % 2 == 1;
    }

    public static Pose2d getPolePose(Translation2d relativeTranslation, int pole) {
        return relativeToFieldAndReflect(relativeTranslation, getRotationOfPole(pole), getMirroringOfPole(pole));
    }

    public static Pose2d getCoralPlacePose(int pole) {
        return getPolePose(PLACE_RELATIVE, pole);
    }

    public static Pose2d getCoralAlignPose(int pole) {
        return getPolePose(ALIGN_RELATIVE, pole);
    }
}
