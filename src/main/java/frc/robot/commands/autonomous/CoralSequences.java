package frc.robot.commands.autonomous;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.MeasureMath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Field.CoralStation;
import frc.robot.Constants.ReefPositioning;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public final class CoralSequences {
  private CoralSequences() {}

  public static enum CoralSource {
    LEFT(CoralStation.LEFT),
    RIGHT(CoralStation.RIGHT),
    HELD(null);

    public final CoralStation station;

    CoralSource(CoralStation station) {
      this.station = station;
    }
  }

  public static record CoralPosition(int pole, int level) {}

  public static record Placement(CoralSource source, CoralPosition target) {
    @Override
    public final String toString() {
      return String.format("%s %d %d", source, target.pole, target.level);
    }

    public final List<Pose2d> getPoses() {
      List<Pose2d> poses = new ArrayList<>();

      if (source != CoralSource.HELD) {
        poses.add(source.station.pose);
      }

      poses.add(ReefPositioning.getCoralAlignPose(target.pole));

      return poses;
    }
  }

  private static SwerveConfig swerveConfig = Constants.SWERVE.CONFIG;

  public static List<Placement> getFastestSequence(
      List<CoralPosition> positions,
      boolean leftStation,
      boolean rightStation,
      boolean startsWithCoral,
      Pose2d startPose) {
    List<List<Placement>> sequences =
        generateSequences(positions, leftStation, rightStation, startsWithCoral);

    double bestScore = Double.POSITIVE_INFINITY;
    List<Placement> bestSequence = null;

    int scanned = 0;
    int total = sequences.size();

    for (List<Placement> sequence : sequences) {
      double score = getTimeBiasedScore(sequence, startPose, 0.5);

      if (score < bestScore) {
        bestScore = score;
        bestSequence = sequence;
      }

      if (total >= 10 && scanned % (total / 10) == 0) {
        System.out.println("Scanned " + scanned + " / " + total + " routes");
      }

      scanned++;
    }

    if (bestSequence == null) {
      DriverStation.reportError("No autonomous sequence", true);
      System.out.println("No autonomous sequence!");
    } else {
      System.out.println(sequences.size() + " routes scanned.");
      System.out.println(
          "Best: "
              + bestScore
              + " inefficiency, "
              + getSequenceTime(bestSequence, startPose).in(Seconds)
              + " ideal seconds");
    }

    return bestSequence;
  }

  public static void displaySequence(
      String name, Collection<Placement> sequence, Pose2d startPose) {
    List<Pose2d> poses = new ArrayList<>();
    poses.add(startPose);

    for (Placement placement : sequence) {
      List<Pose2d> placementPoses = placement.getPoses();

      poses.addAll(placementPoses);
    }

    Logger.getField().getObject(name).setPoses(poses);
  }

  // SEQUENCE GENERATION

  private static List<List<Placement>> generateSequences(
      List<CoralPosition> positions,
      boolean leftStation,
      boolean rightStation,
      boolean startsWithCoral) {
    if (startsWithCoral) {
      return possibleSequencesWithHeld(positions, leftStation, rightStation);
    } else {
      return possibleSequencesWithoutHeld(positions, leftStation, rightStation);
    }
  }

  private static List<List<Placement>> possibleSequencesWithHeld(
      List<CoralPosition> positions, boolean leftStation, boolean rightStation) {
    List<List<Placement>> sequences = new ArrayList<>();

    for (int i = 0; i < positions.size(); i++) {
      CoralPosition heldPosition = positions.get(i);
      Placement heldPlacement = new Placement(CoralSource.HELD, heldPosition);

      List<CoralPosition> otherPositions = new ArrayList<CoralPosition>(positions);
      otherPositions.remove(i);

      List<List<Placement>> otherSequences =
          possibleSequencesWithoutHeld(otherPositions, leftStation, rightStation);

      for (List<Placement> otherSequence : otherSequences) {
        List<Placement> newSequence = new ArrayList<>();
        newSequence.add(heldPlacement);
        newSequence.addAll(otherSequence);
        sequences.add(newSequence);
      }
    }

    return sequences;
  }

  private static List<List<Placement>> possibleSequencesWithoutHeld(
      List<CoralPosition> positions, boolean leftStation, boolean rightStation) {
    List<List<CoralPosition>> positionPermutations =
        generatePermutations(new ArrayList<>(positions));
    List<List<Placement>> sequences = new ArrayList<>();

    for (List<CoralPosition> sequencePositions : positionPermutations) {
      if (leftStation && rightStation)
        sequences.addAll(placementsFromPositionsEitherSource(sequencePositions));
      else {
        CoralSource source = leftStation ? CoralSource.LEFT : CoralSource.RIGHT;
        ArrayList<Placement> sequence = new ArrayList<>();

        for (CoralPosition position : sequencePositions) {
          sequence.add(new Placement(source, position));
        }

        sequences.add(sequence);
      }
    }

    return sequences;
  }

  private static List<List<Placement>> placementsFromPositionsEitherSource(
      List<CoralPosition> positions) {
    List<List<Placement>> placements = new ArrayList<>();

    placements.addAll(placementsFromPositionsEitherSource(positions, CoralSource.LEFT));
    placements.addAll(placementsFromPositionsEitherSource(positions, CoralSource.RIGHT));

    return placements;
  }

  private static List<List<Placement>> placementsFromPositionsEitherSource(
      List<CoralPosition> positions, CoralSource firstSource) {
    Placement firstPlacement = new Placement(firstSource, positions.get(0));
    List<List<Placement>> placements = new ArrayList<>();

    if (positions.size() == 1) {
      return List.of(List.of(firstPlacement));
    }

    List<List<Placement>> otherPlacements = new ArrayList<>();

    otherPlacements.addAll(
        placementsFromPositionsEitherSource(positions.subList(1, positions.size())));

    for (List<Placement> otherPlacement : otherPlacements) {
      List<Placement> newPlacement = new ArrayList<>();
      newPlacement.add(firstPlacement);
      newPlacement.addAll(otherPlacement);
      placements.add(newPlacement);
    }

    return placements;
  }

  // https://stackoverflow.com/questions/10305153/generating-all-possible-permutations-of-a-list-recursively
  private static <E> List<List<E>> generatePermutations(List<E> original) {
    if (original.isEmpty()) {
      List<List<E>> result = new ArrayList<>();
      result.add(new ArrayList<>());
      return result;
    }

    E firstElement = original.remove(0);
    List<List<E>> returnValue = new ArrayList<>();
    List<List<E>> permutations = generatePermutations(original);

    for (List<E> smallerPermutated : permutations) {
      for (int index = 0; index <= smallerPermutated.size(); index++) {
        List<E> temp = new ArrayList<>(smallerPermutated);
        temp.add(index, firstElement);
        returnValue.add(temp);
      }
    }

    return returnValue;
  }

  // TIME ESTIMATION

  public static Time getSequenceTime(List<Placement> placements, Pose2d startPose) {
    MutTime time = Seconds.mutable(0);

    Pose2d currentPose = startPose;

    for (Placement placement : placements) {
      Pair<Time, Pose2d> result = getPlacementResult(placement, currentPose);

      time.mut_plus(result.getFirst());
      currentPose = result.getSecond();
    }

    return time;
  }

  public static double getTimeBiasedScore(
      List<Placement> placements, Pose2d startPose, double timeBias) {
    MutTime time = Seconds.mutable(0);
    double bias = 0;

    Pose2d currentPose = startPose;

    for (Placement placement : placements) {
      Pair<Time, Pose2d> result = getPlacementResult(placement, currentPose);

      time.mut_plus(result.getFirst());

      bias += timeBias * time.in(Seconds);

      currentPose = result.getSecond();
    }

    return time.in(Seconds) + bias;
  }

  public static Pair<Time, Pose2d> getPlacementResult(Placement placement, Pose2d startPose) {
    MutTime sequenceTime = Seconds.mutable(0);

    Pose2d currentPose = startPose;

    if (placement.source != CoralSource.HELD) {
      Pose2d stationPose = placement.source().station.pose;
      sequenceTime.mut_plus(estimatePathTime(currentPose, stationPose));

      currentPose = stationPose;
    }

    sequenceTime.mut_plus(
        estimatePathTime(currentPose, ReefPositioning.getCoralAlignPose(placement.target.pole())));

    currentPose = ReefPositioning.getCoralAlignPose(placement.target.pole());

    return Pair.of(sequenceTime, currentPose);
  }

  public static Distance ESTIMATED_REEF_AVOIDANCE_DIAMETER = Inches.of(110);

  private static double estimatePathTranslationLength(double x1, double x2, double y, double r) {
    double t = Math.asin(Math.abs(y / r));

    return 2 * (Math.PI / 2 - t) * r
        + Math.abs(x2 - r * Math.cos(t))
        + Math.abs(x1 + r * Math.cos(t));
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

    if (Math.abs(y) > r || Math.signum(x1) == Math.signum(x2))
      return Meters.of(start.getDistance(end));

    return Meters.of(estimatePathTranslationLength(x1, x2, y, r));
  }

  private static Distance estimateModuleMovement(Pose2d start, Pose2d end) {
    return estimatePathTranslationLength(start.getTranslation(), end.getTranslation())
        .plus(
            swerveConfig
                .chassis()
                .driveRadius()
                .times(
                    MeasureMath.abs(
                            MeasureMath.minDifference(start.getRotation(), end.getRotation()))
                        .getRadians()));
  }

  public static Time estimatePathTime(Pose2d start, Pose2d end) {
    return estimateModuleMovement(start, end).div(swerveConfig.maxDriveSpeed());
  }
}
