package frc.robot.auto.utils;

import static edu.wpi.first.units.Units.Degrees;

import java.util.LinkedList;
import java.util.List;

import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Field.CoralStation;
import frc.robot.util.software.Dashboard.AutonChooser;

public final class AutoPaths {
    public static class CoralPosition {
        /**
         * Coral pole ranging from 0-11.
         */
        public final int pole;

        /**
         * Coral level ranging from 1-4.
         */
        public final int level;

        public CoralPosition(int pole, int level) {
            if (pole < 0 || pole > 11) {
                pole = (pole % 12 + 12) % 12;
            }

            if (level < 1 || level > 4) {
                System.out.println("Coral level is out of range");
                level = 2;
            }

            this.pole = pole;
            this.level = level;
        }

        @Override
        public boolean equals(Object obj) {
            return obj instanceof CoralPosition && ((CoralPosition) obj).pole == pole && ((CoralPosition) obj).level == level;
        }
    }

    public static class CoralSources {
        public final boolean preplaced;
        public final boolean leftStation;
        public final boolean rightStation;
        public final int count;

        public CoralSources(boolean preplaced, boolean leftStation, boolean rightStation) {
            this.preplaced = preplaced;
            this.leftStation = leftStation;
            this.rightStation = rightStation;
            count = (preplaced ? 1 : 0) + (leftStation ? 1 : 0) + (rightStation ? 1 : 0);
        }

        @Override
        public boolean equals(Object obj) {
            return obj instanceof CoralSources && ((CoralSources) obj).preplaced == preplaced && ((CoralSources) obj).leftStation == leftStation && ((CoralSources) obj).rightStation == rightStation;
        }
    }

    public static class PlanConstraints {
        public final List<CoralPosition> targets;
        public final CoralSources sources;
        
        public PlanConstraints(List<CoralPosition> targets, CoralSources sources) {
            this.targets = targets;
            this.sources = sources;
        }

        public boolean pathExists() {
            return targets.size() > 0 && sources.count > 0;
        }

        public boolean equals(PlanConstraints other) {
            return other != null && other.targets.equals(targets) && other.sources.equals(sources);
        }
    }

    public static class PlanParameters {
        public final PlanConstraints constraints;
        public final Pose2d startPose;

        public PlanParameters(PlanConstraints constraints, Pose2d startPose) {
            this.constraints = constraints;
            this.startPose = startPose;
        }

        public boolean matches(PlanParameters other) {
            boolean a = other != null;
            boolean b = other.constraints.equals(constraints);
            boolean c = other.startPose.getTranslation().getDistance(startPose.getTranslation()) < Units.inchesToMeters(6);
            boolean d = MeasureMath.minDifference(other.startPose.getRotation(), startPose.getRotation()).getMeasure().abs(Degrees) < 10;

            return a && b && c && d;
        }

        public static PlanParameters fromAutoChooser(boolean preplaced, Pose2d startPose) {
            CoralSources sources = new CoralSources(preplaced, AutonChooser.leftCoralStation(), AutonChooser.rightCoralStation());
            
            List<CoralPosition> targets = new LinkedList<>();

            for (int i : AutonChooser.reefFaces()) {
                int pole1 = (6 - i * 2 + 12) % 12; // ((6 - i * 2) % 12 + 12) % 12
                int pole2 = (5 - i * 2 + 12) % 12;

                targets.add(new CoralPosition(pole1, 4));
                targets.add(new CoralPosition(pole2, 4));
            }

            return new PlanParameters(new PlanConstraints(targets, sources), startPose);
        }
    }

    public static enum CoralSource {
        STATION_LEFT,
        STATION_RIGHT,
        PREPLACED;

        public static CoralSource of(CoralStation station) {
            return station == CoralStation.LEFT ? STATION_LEFT : STATION_RIGHT;
        }

        public CoralStation station() {
            return this == STATION_LEFT ? CoralStation.LEFT : CoralStation.RIGHT;
        }
    }

    public static class CoralMovement {
        public final CoralPosition destination;
        public final CoralSource source;

        public CoralMovement(CoralPosition destination, CoralSource source) {
            this.destination = destination;
            this.source = source;
        }

        @Override
        public boolean equals(Object obj) {
            return obj instanceof CoralMovement && ((CoralMovement) obj).destination.equals(destination) && ((CoralMovement) obj).source == source;
        }
    }

    private AutoPaths() {}

    public static final class Logging {
        public static final String AUTO = "Autonomous";
        public static final String SEQUENCE_CHOOSER = AUTO + "/SequenceChooser";
        public static final String COMMAND_BUILDER = AUTO + "/CommandBuilder";
        public static final String AUTO_THREAD = AUTO + "/AutoThread";
        public static final String AUTO_GENERATION = AUTO + "/AutoGeneration";

        private Logging() {}
    }
}
