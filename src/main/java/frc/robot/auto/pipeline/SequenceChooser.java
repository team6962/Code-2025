package frc.robot.auto.pipeline;

import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import frc.robot.auto.utils.AutoPaths;
import frc.robot.auto.utils.PathOutlineFactory;
import frc.robot.auto.utils.PathTiming;
import java.util.List;

public class SequenceChooser {
  private PathOutlineFactory pathFactory;
  private List<AutoPaths.CoralMovement> bestPath;
  private Time bestTime;
  private Pose2d startPose;

  public void start(AutoPaths.PlanParameters parameters) {
    pathFactory = new PathOutlineFactory(parameters.constraints);
    this.startPose = parameters.startPose;
    bestPath = null;
    bestTime = Seconds.of(Double.POSITIVE_INFINITY);
    // Logger.log(AutoPaths.Logging.SEQUENCE_CHOOSER + "/isDone", false);
    // Logger.log(AutoPaths.Logging.SEQUENCE_CHOOSER + "/pathCount", pathFactory.getPathCount());
  }

  public boolean isDone() {
    return pathFactory != null && !pathFactory.hasNext();
  }

  public void work() {
    if (isDone()) return;

    List<AutoPaths.CoralMovement> path = pathFactory.next();
    Time time = PathTiming.getPathTime(path, startPose);

    if (isDone()) {
      // Logger.log(AutoPaths.Logging.SEQUENCE_CHOOSER + "/isDone", true);
    }

    if (time.lt(bestTime)) {
      bestTime = time;
      bestPath = path;

      // Logger.log(AutoPaths.Logging.SEQUENCE_CHOOSER + "/bestTime", bestTime);
      Logger.logObject(AutoPaths.Logging.SEQUENCE_CHOOSER + "/bestPath", bestPath);
    }

    int pathIndex = pathFactory.getCurrentIndex();

    if (Math.abs(((double) pathIndex * 1000 / pathFactory.getPathCount()) % 1) < 1e-6) {
      // Logger.log(AutoPaths.Logging.SEQUENCE_CHOOSER + "/pathIndex", pathIndex);
    }
  }

  public List<AutoPaths.CoralMovement> getBestPath() {
    return bestPath;
  }

  public Time getBestTime() {
    return bestTime;
  }
}
