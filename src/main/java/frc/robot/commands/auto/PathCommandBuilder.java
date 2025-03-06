package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.Autonomous;

public class PathCommandBuilder {
    private Autonomous autonomous;
    private SequentialCommandGroup group = new SequentialCommandGroup();
    private List<AutoPaths.CoralMovement> path;
    private int currentIndex = 0;

    public PathCommandBuilder(Autonomous autonomous) {
        this.autonomous = autonomous;
    }

    public void start(List<AutoPaths.CoralMovement> path) {
        group = new SequentialCommandGroup();
        this.path = path;
    }

    public void work() {
        AutoPaths.CoralMovement movement = path.get(currentIndex);

        if (movement.source != AutoPaths.CoralSource.PREPLACED) {
            group.addCommands(autonomous.intakeCoral(movement.source.station()));
        }

        group.addCommands(autonomous.placeCoral(movement.destination));

        currentIndex++;
    }

    public boolean isDone() {
        return currentIndex + 1 >= path.size();
    }

    public Command getCommand() {
        return group;
    }
}
