package frc.robot.auto.pipeline;

import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ReefPositioning;
import frc.robot.Constants.StationPositioning;
import frc.robot.auto.utils.AutoCommands;
import frc.robot.auto.utils.AutoPaths;
import java.util.List;

public class CommandBuilder {
  private AutoCommands autonomous;
  private SequentialCommandGroup group = new SequentialCommandGroup();
  private List<AutoPaths.CoralMovement> path;
  private int currentIndex = 0;
  private SequenceChooser sequenceChooser;
  private int logCommandIndex = 0;
  private Pose2d previousPose;

  public CommandBuilder(AutoCommands autonomous) {
    this.autonomous = autonomous;
    sequenceChooser = new SequenceChooser();
  }

  public void start(AutoPaths.PlanParameters parameters) {
    sequenceChooser.start(parameters);
    group = new SequentialCommandGroup();
    currentIndex = 0;
    path = null;
    previousPose = parameters.startPose;
    logCommandIndex = 0;

    System.out.println("Starting to generate a new autonomous path");

    // Logger.log(AutoPaths.Logging.COMMAND_BUILDER + "/isDone", false);
    // Logger.log(AutoPaths.Logging.COMMAND_BUILDER + "/currentIndex", 0);
    // Logger.log(AutoPaths.Logging.COMMAND_BUILDER + "/status", "started");
  }

  public void work() {
    if (!sequenceChooser.isDone()) {
      // Logger.log(AutoPaths.Logging.COMMAND_BUILDER + "/status", "waiting");
      sequenceChooser.work();
      return;
    }

    // Logger.log(AutoPaths.Logging.COMMAND_BUILDER + "/workTimestamp", Timer.getFPGATimestamp());
    // Logger.log(AutoPaths.Logging.COMMAND_BUILDER + "/status", "running");

    if (path == null) {
      path = sequenceChooser.getBestPath();
    }

    if (currentIndex + 1 > path.size()) return;

    AutoPaths.CoralMovement movement = path.get(currentIndex);

    if (movement.source != AutoPaths.CoralSource.PREPLACED) {
      group.addCommands(autonomous.intakeCoral(previousPose, movement.source.station()));

      previousPose = StationPositioning.getIntakePose(movement.source.station(), 4);

      Logger.log(
          AutoPaths.Logging.COMMAND_BUILDER + "/commands/" + logCommandIndex,
          "intakeCoral(" + movement.source.station().name() + ")");
      logCommandIndex++;
    }

    group.addCommands(autonomous.placeCoral(previousPose, movement.destination));
    previousPose = ReefPositioning.getCoralPlacePose(movement.destination.pole);
    Logger.log(
        AutoPaths.Logging.COMMAND_BUILDER + "/commands/" + logCommandIndex,
        "placeCoral(level "
            + movement.destination.level
            + ", pole "
            + movement.destination.pole
            + ")");
    logCommandIndex++;

    currentIndex++;

    // Logger.log(AutoPaths.Logging.COMMAND_BUILDER + "/currentIndex", currentIndex);
    // Logger.log(AutoPaths.Logging.COMMAND_BUILDER + "/isDone", isDone());
    // Logger.logObject(AutoPaths.Logging.COMMAND_BUILDER + "/pathCommand", getPathCommand());
  }

  public boolean isDone() {
    return sequenceChooser.isDone() && path != null && currentIndex + 1 > path.size();
  }

  public Command getPathCommand() {
    return group;
  }
}
