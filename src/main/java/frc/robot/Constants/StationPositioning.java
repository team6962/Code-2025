package frc.robot.Constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.Constants.SWERVE;
import frc.robot.Constants.Field.CoralStation;

public final class StationPositioning {
    private StationPositioning() {}

    private static final Rotation2d STATION_FRONT_ANGLE = Rotation2d.fromDegrees(144.011392);
    private static final Distance STATION_OFFSET_X = Inches.of(65.828076);
    private static final Translation2d STATION_EDGE = new Translation2d(STATION_OFFSET_X.in(Meters), 0);
    private static final Distance FIRST_SLOT_OFFSET = Inches.of(7.8775);
    private static final Distance OFFSET_BETWEEN_SLOTS = Inches.of(8);
    public static final int SLOT_COUNT = 9;

    public static Translation2d getSlotTranslation(int rsSlotIndex) {
        return STATION_EDGE.plus(
            new Translation2d(FIRST_SLOT_OFFSET.plus(OFFSET_BETWEEN_SLOTS.times(rsSlotIndex)).in(Meters), 0)
                .rotateBy(STATION_FRONT_ANGLE)
        );
    }

    public static Pose2d getPlacePose(Translation2d rsSlotTranslation) {
        Pose2d relativePosition = new Pose2d(new Translation2d(SWERVE.CONFIG.chassis().outerLength().div(2).plus(Inches.of(2)).in(Meters), 0), Rotation2d.fromDegrees(0));

        relativePosition = relativePosition.rotateBy(STATION_FRONT_ANGLE.minus(Rotation2d.fromDegrees(90)));
        relativePosition = new Pose2d(relativePosition.getTranslation().plus(rsSlotTranslation), relativePosition.getRotation());

        return relativePosition;
    }

    public static Pose2d getPlacePose(boolean reflect, int slotIndex) {
        Pose2d rsPose = getPlacePose(getSlotTranslation(slotIndex));

        if (reflect) {
            rsPose = new Pose2d(rsPose.getX(), Field.WIDTH - rsPose.getY(), rsPose.getRotation().unaryMinus());
        }

        return rsPose;
    }

    public static Pose2d getPlacePose(CoralStation coralStation, int slotIndex) {
        return getPlacePose(coralStation == CoralStation.LEFT, slotIndex);
    }

    public static Pose2d getNearestIntakePose(CoralStation coralStation, Pose2d currentPose) {
        return getPlacePose(coralStation, 4);
    }

    public static Pose2d getCenterIntakePose(boolean reflect) {
        return getPlacePose(reflect, 4);
    }

    public static Pose2d getCenterIntakePose(CoralStation coralStation) {
        return getCenterIntakePose(coralStation == CoralStation.LEFT);
    }
}
