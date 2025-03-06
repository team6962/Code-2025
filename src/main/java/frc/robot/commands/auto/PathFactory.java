package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants.Field.CoralStation;
import frc.robot.commands.auto.AutoPaths.CoralPosition;
import frc.robot.commands.auto.AutoPaths.CoralSource;

public class PathFactory {
    private AutoPaths.PlanConstraints constraints;

    public PathFactory(AutoPaths.PlanConstraints constraints) {
        this.constraints = constraints;
    }

    /**
     * Until there are no remaining destinations or sources, choose a destination
     * and a source. Remove a destination and, if the source is preplaced, remove it.
     */
    public int getPathCount() {
        int count = 1;

        int sources = constraints.sources.count;
        boolean preplaced = constraints.sources.preplaced;
        int destinations = constraints.targets.size();

        while (sources > 0 && destinations > 0) {
            if (preplaced) {
                sources--;
                preplaced = false;
            } else {
                count *= sources;
            }

            count *= destinations;
            destinations--;
        }

        return count;
    }

    public List<AutoPaths.CoralMovement> getPath(int index) {
        boolean preplaced = constraints.sources.preplaced;
        List<Integer> available = new ArrayList<Integer>(constraints.targets.size());

        for (int i = 0; i < index; i++) {
            available.add(i);
        }

        List<AutoPaths.CoralMovement> path = new ArrayList<>();

        int i = index;

        if (preplaced) {
            int destinationIndex = i % available.size();
            i /= available.size();
            available.remove(destinationIndex);
            path.add(new AutoPaths.CoralMovement(constraints.targets.get(available.get(destinationIndex)), CoralSource.PREPLACED));
        }

        if (!constraints.sources.leftStation && !constraints.sources.rightStation) return path;

        while (available.size() > 0) {
            CoralStation sourceStation;

            if (constraints.sources.leftStation && constraints.sources.rightStation) {
                int sourceIndex = i % 2;
                i /= 2;
                sourceStation = sourceIndex == 0 ? CoralStation.LEFT : CoralStation.RIGHT;
            } else if (constraints.sources.leftStation) {
                sourceStation = CoralStation.LEFT;
            } else {
                sourceStation = CoralStation.RIGHT;
            }

            int destinationIndex = i % available.size();
            i /= available.size();
            available.remove(destinationIndex);

            CoralPosition destination = constraints.targets.get(available.get(destinationIndex));

            path.add(new AutoPaths.CoralMovement(destination, AutoPaths.CoralSource.of(sourceStation)));
        }

        return path;
    }
}
