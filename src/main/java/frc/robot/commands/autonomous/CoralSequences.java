package frc.robot.commands.autonomous;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Field;
import frc.robot.Constants.ReefPositioning;
import frc.robot.Constants.Field.CoralStation;

public final class CoralSequences {
    private CoralSequences() {}

    public static enum CoralSource {
        LEFT(CoralStation.LEFT), RIGHT(CoralStation.RIGHT), HELD(null);

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
        List<CoralPosition> positions, boolean leftStation,
        boolean rightStation, boolean startsWithCoral, Pose2d startPose
    ) {
        List<List<Placement>> sequences = generateSequences(positions, leftStation, rightStation, startsWithCoral);

        Time bestTime = Seconds.of(Double.POSITIVE_INFINITY);
        List<Placement> bestSequence = null;

        for (List<Placement> sequence : sequences) {
            Time time = getSequenceTime(sequence, startPose);

            if (time.lt(bestTime)) {
                bestTime = time;
                bestSequence = sequence;
            }
        }

        Logger.getField().getObject("Autonomous Sequence")
            .setPoses(bestSequence.stream().map(placement -> placement.getPoses()).flatMap(List::stream).toList());

        return bestSequence;
    }

    // SEQUENCE GENERATION
    
    private static List<List<Placement>> generateSequences(
        List<CoralPosition> positions, boolean leftStation,
        boolean rightStation, boolean startsWithCoral
    ) {
        if (startsWithCoral) {
            return possibleSequencesWithHeld(positions, leftStation, rightStation);
        } else {
            return possibleSequencesWithoutHeld(positions, leftStation, rightStation);
        }
    }

    private static List<List<Placement>> possibleSequencesWithHeld(List<CoralPosition> positions, boolean leftStation, boolean rightStation) {
        List<List<Placement>> sequences = new ArrayList<>();

        for (int i = 0; i < positions.size(); i++) {
            CoralPosition heldPosition = positions.get(i);
            Placement heldPlacement = new Placement(CoralSource.HELD, heldPosition);

            List<CoralPosition> otherPositions = new ArrayList<CoralPosition>(positions);
            otherPositions.remove(i);

            List<List<Placement>> otherSequences = possibleSequencesWithoutHeld(otherPositions, leftStation, rightStation);

            for (List<Placement> otherSequence : otherSequences) {
                List<Placement> newSequence = new ArrayList<>();
                newSequence.add(heldPlacement);
                newSequence.addAll(otherSequence);
                sequences.add(newSequence);
            }
        }

        return sequences;
    }

    private static List<List<Placement>> possibleSequencesWithoutHeld(List<CoralPosition> positions, boolean leftStation, boolean rightStation) {
        List<List<CoralPosition>> positionPermutations = generatePermutations(new ArrayList<>(positions));
        List<List<Placement>> sequences = new ArrayList<>();

        for (List<CoralPosition> sequencePositions : positionPermutations) {
            if (leftStation && rightStation) sequences.addAll(placementsFromPositionsEitherSource(sequencePositions));
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
    
    private static List<List<Placement>> placementsFromPositionsEitherSource(List<CoralPosition> positions) {
        List<List<Placement>> placements = new ArrayList<>();

        placements.addAll(placementsFromPositionsEitherSource(positions, CoralSource.LEFT));
        placements.addAll(placementsFromPositionsEitherSource(positions, CoralSource.RIGHT));

        return placements;
    }

    private static List<List<Placement>> placementsFromPositionsEitherSource(List<CoralPosition> positions, CoralSource firstSource) {
        Placement firstPlacement = new Placement(firstSource, positions.get(0));
        List<List<Placement>> placements = new ArrayList<>();

        if (positions.size() == 1) {
            return List.of(List.of(firstPlacement));
        }

        List<List<Placement>> otherPlacements = new ArrayList<>();

        otherPlacements.addAll(placementsFromPositionsEitherSource(positions.subList(1, positions.size()), CoralSource.LEFT));
        otherPlacements.addAll(placementsFromPositionsEitherSource(positions.subList(1, positions.size()), CoralSource.RIGHT));

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
            MutTime sequenceTime = Seconds.mutable(0);

            if (placement.source != CoralSource.HELD) {
                Pose2d stationPose = placement.source == CoralSource.LEFT ? Field.getLeftCoralStationPose() : Field.getRightCoralStationPose();
                sequenceTime.mut_plus(estimatePathTime(currentPose, stationPose));

                currentPose = stationPose;
            }

            sequenceTime.mut_plus(estimatePathTime(currentPose, ReefPositioning.getCoralAlignPose(placement.target.pole())));

            time.mut_plus(sequenceTime);
            currentPose = ReefPositioning.getCoralAlignPose(placement.target.pole());
        }

        return time;
    }

    public static Distance ESTIMATED_REEF_AVOIDANCE_DIAMETER = Inches.of(116);

    public static Distance estimatePathTranslationLength(Translation2d start, Translation2d end) {
        start = start.minus(ReefPositioning.REEF_CENTER);
        end = end.minus(ReefPositioning.REEF_CENTER);

        Rotation2d lineRotation = end.minus(start).getAngle();
        Translation2d startRotated = start.rotateBy(lineRotation.unaryMinus());
        double lineDistance = startRotated.getY();
        double reefRadius = ESTIMATED_REEF_AVOIDANCE_DIAMETER.in(Meters) / 2.0;
        double ratio = Math.abs(lineDistance / reefRadius);

        if (ratio > 1) return Meters.of(end.minus(start).getNorm());

        double arcAngle = reefRadius * (Math.PI / 2 - Math.asin(ratio));
        double arcLength = 2 * arcAngle;
        double chordLength = 2 * Math.cos(Math.asin(ratio));

        return Meters.of(end.minus(start).getNorm() - chordLength + arcLength);
    }

    private static Distance estimateModuleMovement(Pose2d start, Pose2d end) {
        return estimatePathTranslationLength(start.getTranslation(), end.getTranslation())
            .plus(swerveConfig.chassis().driveRadius().times(
                MeasureMath.abs(MeasureMath.minDifference(start.getRotation(), end.getRotation())).getRadians()
            ));
    }

    private static Time estimatePathTime(Pose2d start, Pose2d end) {
        return estimateModuleMovement(start, end).div(swerveConfig.maxDriveSpeed());
    }
}
