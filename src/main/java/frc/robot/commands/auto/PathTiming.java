package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.Constants.SWERVE;
import frc.robot.Constants.ReefPositioning;

public final class PathTiming {
    public static Time getPathTime(List<AutoPaths.CoralMovement> path, Pose2d startPose) {
        return getPathTime(path, startPose, 0);
    }

    public static Time getPathTime(List<AutoPaths.CoralMovement> path, Pose2d startPose, double timeBias) {
        MutTime time = Seconds.mutable(0);

        Pose2d currentPose = startPose;

        for (AutoPaths.CoralMovement movements : path) {
            Pair<Time, Pose2d> result = getMovementResult(movements, currentPose);

            time.mut_plus(result.getFirst());
            
            time.mut_plus(time.times(timeBias));
            
            currentPose = result.getSecond();
        }

        return time;
    }

    public static Pair<Time, Pose2d> getMovementResult(AutoPaths.CoralMovement movement, Pose2d startPose) {
        MutTime sequenceTime = Seconds.mutable(0);

        Pose2d currentPose = startPose;

        if (movement.source != AutoPaths.CoralSource.PREPLACED) {
            Pose2d stationPose = movement.source.station().pose;
            sequenceTime.mut_plus(estimatePathTime(currentPose, stationPose));

            currentPose = stationPose;
        }

        sequenceTime.mut_plus(estimatePathTime(currentPose, ReefPositioning.getCoralAlignPose(movement.destination.pole)));

        currentPose = ReefPositioning.getCoralAlignPose(movement.destination.pole);

        return Pair.of(sequenceTime, currentPose);
    }

    public static Distance ESTIMATED_REEF_AVOIDANCE_DIAMETER = Inches.of(110);

    private static double estimatePathTranslationLength(double x1, double x2, double y, double r) {
        double t = Math.asin(Math.abs(y / r));

        return 2 * (Math.PI / 2 - t) * r + Math.abs(x2 - r * Math.cos(t)) + Math.abs(x1 + r * Math.cos(t));
    }

    public static Distance estimatePathTranslationLength(Translation2d start, Translation2d end) {
        Translation2d startRelative = start.minus(ReefPositioning.REEF_CENTER);
        Translation2d endRelative = end.minus(ReefPositioning.REEF_CENTER);

        Rotation2d lineAngle = endRelative.minus(startRelative).getAngle();
        Translation2d startRotated = startRelative.rotateBy(lineAngle.unaryMinus());
        Translation2d endRotated = endRelative.rotateBy(lineAngle.unaryMinus());

        double x1 = startRotated.getX();
        double x2 = endRotated.getX();
        double y = endRotated.getY();
        double r = ESTIMATED_REEF_AVOIDANCE_DIAMETER.in(Meters) / 2.0;

        if (Math.abs(y) > r || Math.signum(x1) == Math.signum(x2)) return Meters.of(start.getDistance(end));
        
        return Meters.of(estimatePathTranslationLength(x1, x2, y, r));
    }

    private static Distance estimateModuleMovement(Pose2d start, Pose2d end) {
        return estimatePathTranslationLength(start.getTranslation(), end.getTranslation())
            .plus(SWERVE.CONFIG.chassis().driveRadius().times(
                MeasureMath.abs(MeasureMath.minDifference(start.getRotation(), end.getRotation())).getRadians()
            ));
    }

    public static Time estimatePathTime(Pose2d start, Pose2d end) {
        return estimateModuleMovement(start, end).div(SWERVE.CONFIG.maxDriveSpeed());
    }

    private PathTiming() {}
}
