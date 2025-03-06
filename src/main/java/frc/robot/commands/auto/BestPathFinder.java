package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;

public class BestPathFinder {
    private Pose2d startPose;
    private TimedPath bestPath;
    private PathOutlineFactory pathFactory;

    public BestPathFinder(AutoPaths.PlanConstraints constraints, Pose2d startPose) {
        this.startPose = startPose;
        pathFactory = new PathOutlineFactory(constraints);
    }

    public void start(AutoPaths.PlanConstraints constraints, Pose2d startPose) {
        this.startPose = startPose;
        this.bestPath = null;
        pathFactory = new PathOutlineFactory(constraints);
    }

    public boolean isDone() {
        return !pathFactory.hasNext();
    }

    public void work() {
        if (isDone()) return;

        List<AutoPaths.CoralMovement> path = pathFactory.next();
        Time time = PathTiming.getPathTime(path, startPose);

        if (time.lt(bestPath.time)) {
            bestPath = new TimedPath(time, path);
        }
    }

    public List<AutoPaths.CoralMovement> getBestPath() {
        return bestPath.path;
    }

    public static class TimedPath {
        public Time time;
        public List<AutoPaths.CoralMovement> path;

        public TimedPath(Time time, List<AutoPaths.CoralMovement> path) {
            this.time = time;
            this.path = path;
        }
    }
}
