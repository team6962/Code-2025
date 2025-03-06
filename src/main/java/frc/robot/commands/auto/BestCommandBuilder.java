package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autonomous.Autonomous;

public class BestCommandBuilder {
    private BestPathFinder bestPathFinder;
    private PathCommandBuilder pathCommandBuilder;

    public BestCommandBuilder(Autonomous autonomous, AutoPaths.PlanConstraints constraints, Pose2d startPose) {
        bestPathFinder = new BestPathFinder(constraints, startPose);
        pathCommandBuilder = new PathCommandBuilder(autonomous);
    }

    public void start(AutoPaths.PlanConstraints constraints, Pose2d startPose) {
        bestPathFinder.start(constraints, startPose);
        pathCommandBuilder.start(bestPathFinder.getBestPath());
    }

    public void work() {
        if (!bestPathFinder.isDone()) {
            bestPathFinder.work();
        } else {
            pathCommandBuilder.work();
        }
    }

    public Command getCommand() {
        if (!bestPathFinder.isDone()) {
            return null;
        }

        return pathCommandBuilder.getCommand();
    }
}
