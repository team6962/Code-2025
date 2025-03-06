package frc.robot.auto.pipeline;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import frc.robot.auto.utils.AutoPaths;
import frc.robot.auto.utils.PathOutlineFactory;
import frc.robot.auto.utils.PathTiming;

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
    }

    public boolean isDone() {
        return !pathFactory.hasNext();
    }

    public void work() {
        if (isDone()) return;

        List<AutoPaths.CoralMovement> path = pathFactory.next();
        Time time = PathTiming.getPathTime(path, startPose);

        if (time.lt(bestTime)) {
            bestTime = time;
            bestPath = path;
        }
    }

    public List<AutoPaths.CoralMovement> getBestPath() {
        return bestPath;
    }

    public Time getBestTime() {
        return bestTime;
    }
}
