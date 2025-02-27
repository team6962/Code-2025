package frc.robot.commands.autonomous;

import java.util.Collection;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.AutoGeneration.AutoParams;
import frc.robot.commands.autonomous.CoralSequences.CoralSource;
import frc.robot.commands.autonomous.CoralSequences.Placement;

public class GeneratedAuto {
    private Command command;
    private Autonomous autonomous;
    private AutoParams params;
    private List<Placement> sequence;

    public GeneratedAuto(Autonomous autonomous, AutoParams params) {
        this.autonomous = autonomous;
        this.params = params;
    }

    public void setup() {
        if (command != null) return;

        this.sequence = CoralSequences.getFastestSequence(
            params.targetPositions(), params.leftStation(), params.rightStation(),
            params.hasCoral(), params.startPose()
        );

        checkSequence();

        SequentialCommandGroup group = new SequentialCommandGroup();
        
        for (Placement placement : sequence) {
            group.addCommands(placeCoral(placement));
        }

        command = group;
    }

    private void checkSequence() {
        if (sequence == null || sequence.size() == 0) {
            DriverStation.reportError("Failed to generate autononous sequence", true);
            command = Commands.print("Failed to run nonexistent sequence");

            return;
        }
    }

    private Command placeCoral(Placement placement) {
        SequentialCommandGroup group = new SequentialCommandGroup();

        if (placement.source() != CoralSource.HELD) {
            group.addCommands(autonomous.intakeCoral(placement.source().station));
        }

        group.addCommands(autonomous.placeCoral(placement.target()));

        return group;
    }

    public Collection<Placement> getSequence() {
        return sequence;
    }

    public Command getCommand() {
        if (command != null) {
            CommandScheduler.getInstance().removeComposedCommand(command);
        }

        return command;
    }

    public AutoParams getParameters() {
        return params;
    }
}
