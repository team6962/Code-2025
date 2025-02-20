package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.List;

import com.team6962.lib.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.CoralSequences.CoralPosition;
import frc.robot.commands.autonomous.CoralSequences.CoralSource;
import frc.robot.commands.autonomous.CoralSequences.Placement;
import frc.robot.subsystems.manipulator.coral.CoralGrabber;
import frc.robot.util.software.Dashboard.AutonChooser;

public class GeneratedAuto extends Command {
    private List<CoralPosition> targetPositions;
    private boolean leftStation;
    private boolean rightStation;
    private List<Placement> sequence;
    private Command command;
    private Autonomous autonomous;
    private SwerveDrive swerveDrive;
    private CoralGrabber coralGrabber;

    public GeneratedAuto(Autonomous autonomous, SwerveDrive swerveDrive, CoralGrabber coralGrabber) {
        this.autonomous = autonomous;
        this.swerveDrive = swerveDrive;
        this.coralGrabber = coralGrabber;
    }

    public void setup(Pose2d startPose, boolean hasCoral) {
        if (targetPositions == null) configureAutonomous();
        if (sequence == null) generateSequence(startPose, hasCoral);
        if (command == null) generateCommand();
    }

    public void configureAutonomous(List<CoralPosition> targetPositions, boolean leftStation, boolean rightStation) {
        this.targetPositions = targetPositions;
        this.leftStation = leftStation;
        this.rightStation = rightStation;
    }

    public void configureAutonomous() {
        List<CoralPosition> targetPositions = new ArrayList<>();

        for (Integer face : AutonChooser.reefFaces()) {
            targetPositions.add(new CoralPosition(face * 2 - 1, 4));
            targetPositions.add(new CoralPosition(face * 2, 4));
        }
    }

    public void generateSequence(Pose2d startPose, boolean hasCoral) {
        List<Placement> sequence = CoralSequences.getFastestSequence(targetPositions, leftStation, rightStation, hasCoral, startPose);

        if (sequence == null) {
            DriverStation.reportError("No autonomous sequence (after sequence generation)", true);

            return;
        }

        this.sequence = sequence;
    }

    public void generateCommand() {
        if (sequence.size() == 0) {
            DriverStation.reportError("No autonomous sequence (before command generation)", true);
        }

        SequentialCommandGroup group = new SequentialCommandGroup();
        
        for (Placement placement : sequence) {
            group.addCommands(placeCoral(placement));
        }

        command = group;
    }

    private Command placeCoral(Placement placement) {
        SequentialCommandGroup group = new SequentialCommandGroup();

        if (placement.source() != CoralSource.HELD) {
            group.addCommands(autonomous.intakeCoral(placement.source().station));
        }

        group.addCommands(autonomous.placeCoral(placement.target()));

        return group;
    }

    public List<Placement> getSequence() {
        return sequence;
    }

    public Command getCommand() {
        return command;
    }

    @Override
    public void initialize() {
        setup(swerveDrive.getEstimatedPose(), coralGrabber.hasGamePiece());

        CommandScheduler.getInstance().registerComposedCommands(command);

        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
}
