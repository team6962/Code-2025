package frc.robot.auto.pipeline;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.utils.AutoPaths;
import frc.robot.auto.utils.AutonomousCommands;

public class CommandBuilder {
    private AutonomousCommands autonomous;
    private SequentialCommandGroup group = new SequentialCommandGroup();
    private List<AutoPaths.CoralMovement> path;
    private int currentIndex = 0;
    private SequenceChooser sequenceChooser;

    public CommandBuilder(AutonomousCommands autonomous) {
        this.autonomous = autonomous;
    }

    public void start(AutoPaths.PlanParameters parameters) {
        sequenceChooser.start(parameters);
        group = new SequentialCommandGroup();
        currentIndex = 0;
        path = null;
    }

    public void work() {
        if (!sequenceChooser.isDone()) {
            sequenceChooser.work();
            return;
        }

        if (path == null) {
            path = sequenceChooser.getBestPath();
        }

        AutoPaths.CoralMovement movement = path.get(currentIndex);

        if (movement.source != AutoPaths.CoralSource.PREPLACED) {
            group.addCommands(autonomous.intakeCoral(movement.source.station()));
        }

        group.addCommands(autonomous.placeCoral(movement.destination));

        currentIndex++;
    }

    public boolean isDone() {
        return sequenceChooser.isDone() && currentIndex + 1 == path.size();
    }

    public Command getPathCommand() {
        return group;
    }
}
