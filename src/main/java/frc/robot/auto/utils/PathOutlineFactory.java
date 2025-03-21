package frc.robot.auto.utils;

import frc.robot.Constants.Field.CoralStation;
import frc.robot.auto.utils.AutoPaths.CoralMovement;
import frc.robot.auto.utils.AutoPaths.CoralPosition;
import frc.robot.auto.utils.AutoPaths.CoralSource;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class PathOutlineFactory implements Iterator<List<AutoPaths.CoralMovement>> {
  private AutoPaths.PlanConstraints constraints;
  private int index = 0;
  private int count = 0;

  public PathOutlineFactory() {
    this(new AutoPaths.PlanConstraints(List.of(), new AutoPaths.CoralSources(false, false, false)));
  }

  public PathOutlineFactory(AutoPaths.PlanConstraints constraints) {
    start(constraints);
  }

  public void start(AutoPaths.PlanConstraints constraints) {
    this.constraints = constraints;
    this.index = 0;
    this.count = computePathCount();
  }

  @Override
  public List<CoralMovement> next() {
    List<CoralMovement> path = getPath(index);
    index++;
    return path;
  }

  @Override
  public boolean hasNext() {
    return index < count;
  }

  public int getPathCount() {
    return count;
  }

  public int getCurrentIndex() {
    return index;
  }

  private int computePathCount() {
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
    if (constraints.sources.count == 0 || constraints.targets.size() == 0) {
      return List.of();
    }

    boolean preplaced = constraints.sources.preplaced;
    List<Integer> available = new ArrayList<Integer>(constraints.targets.size());

    for (int i = 0; i < constraints.targets.size(); i++) {
      available.add(i);
    }

    List<AutoPaths.CoralMovement> path = new ArrayList<>();

    int i = index;

    if (preplaced) {
      int destinationIndex = i % available.size();
      i /= available.size();

      path.add(
          new AutoPaths.CoralMovement(
              constraints.targets.get(available.get(destinationIndex)), CoralSource.PREPLACED));

      available.remove(destinationIndex);
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

      CoralPosition destination = constraints.targets.get(available.get(destinationIndex));

      available.remove(destinationIndex);

      path.add(new AutoPaths.CoralMovement(destination, AutoPaths.CoralSource.of(sourceStation)));
    }

    return path;
  }
}
