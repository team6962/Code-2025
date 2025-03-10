package frc.robot.commands.autonomous;

import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.MeasureMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.autonomous.CoralSequences.CoralPosition;
import frc.robot.util.CachedRobotState;
import frc.robot.util.software.Dashboard.AutonChooser;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoGeneration extends Thread {
  private List<GeneratedAuto> autos = new ArrayList<>();
  private AutoParams currentParams = new AutoParams(List.of(), false, false, new Pose2d(), false);
  private WorkerState state = new WorkerState(false);

  private Autonomous autonomous;

  public AutoGeneration(Autonomous autonomous) {
    this.autonomous = autonomous;
  }

  public AutoGeneration clone() {
    AutoGeneration autoGen = new AutoGeneration(autonomous);

    synchronized (autos) {
      autoGen.autos = autos;
    }

    synchronized (currentParams) {
      autoGen.currentParams = currentParams;
    }

    synchronized (state) {
      autoGen.state = state;
    }

    return autoGen;
  }

  public void setParameters(AutoParams parameters) {
    synchronized (currentParams) {
      currentParams = parameters;
    }
  }

  public AutoParams getParameters() {
    synchronized (currentParams) {
      return currentParams;
    }
  }

  public void setWorking(boolean working) {
    synchronized (state) {
      state = new WorkerState(working);
    }
  }

  private boolean isWorking() {
    synchronized (state) {
      return state.working;
    }
  }

  @Override
  public void run() {
    while (isWorking()) {
      boolean didWork = generate().didWork;

      if (didWork) {
        try {
          sleep(100);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }

    synchronized (autos) {
      autos.clear();
    }
  }

  private WorkResult generate() {
    return generate(getParameters());
  }

  private WorkResult generate(AutoParams params) {
    if (!params.leftStation && !params.rightStation) {
      return new WorkResult(false, null);
    }

    Logger.log("/AutoGeneration/lastGenerationStartTime", Timer.getFPGATimestamp());

    GeneratedAuto newAuto;

    synchronized (autos) {
      for (GeneratedAuto auto : autos) {
        if (auto.getParameters().matches(params)) return new WorkResult(false, auto);
      }
    }

    newAuto = new GeneratedAuto(autonomous, params);
    newAuto.setup();

    synchronized (autos) {
      autos.add(newAuto);
    }

    Logger.log("/AutoGeneration/lastGenerationEndTime", Timer.getFPGATimestamp());

    return new WorkResult(true, newAuto);
  }

  public boolean doneComputing(AutoParams params) {
    if (!params.leftStation && !params.rightStation) return false;

    synchronized (autos) {
      for (GeneratedAuto auto : autos) {
        if (auto.getParameters().matches(params)) return true;
      }
    }

    return false;
  }

  /**
   * @return Can be null!
   */
  public GeneratedAuto get(AutoParams params) {
    return generate(params).auto();
  }

  private static record WorkerState(boolean working) {}

  private static record WorkResult(boolean didWork, GeneratedAuto auto) {}

  public static record AutoParams(
      List<CoralPosition> targetPositions,
      boolean leftStation,
      boolean rightStation,
      Pose2d startPose,
      boolean hasCoral) {
    public boolean matches(AutoParams other) {
      return targetPositions.equals(other.targetPositions)
          && leftStation == other.leftStation
          && rightStation == other.rightStation
          && startPose.getTranslation().getDistance(other.startPose.getTranslation()) < 0.2
          && MeasureMath.minDifference(startPose.getRotation(), other.startPose.getRotation())
                  .getDegrees()
              < 10
          && hasCoral == other.hasCoral;
    }

    public boolean valid() {
      return targetPositions != null && startPose != null && targetPositions.size() > 0;
    }

    public static AutoParams test(Pose2d currentPose) {
      return new AutoParams(
          List.of(
              new CoralSequences.CoralPosition(1, 2),
              new CoralSequences.CoralPosition(3, 4),
              new CoralSequences.CoralPosition(8, 1),
              new CoralSequences.CoralPosition(2, 3),
              new CoralSequences.CoralPosition(11, 1),
              new CoralSequences.CoralPosition(7, 4),
              new CoralSequences.CoralPosition(6, 2)),
          true,
          true,
          currentPose,
          true);
    }

    public static AutoParams get(Pose2d currentPose, boolean hasCoral) {
      List<CoralPosition> positions = new LinkedList<>();

      for (int face : AutonChooser.reefFaces()) {
        positions.add(new CoralPosition(face * 2, 4));
        positions.add(new CoralPosition(((face * 2 - 1) % 12 + 12) % 12, 4));
      }

      return new AutoParams(
          positions,
          AutonChooser.leftCoralStation(),
          AutonChooser.rightCoralStation(),
          currentPose,
          hasCoral);
    }
  }

  public static class Generator {
    private Supplier<Pose2d> poseSupplier;
    private BooleanSupplier hasCoralSupplier;
    private AutoGeneration autoGen;
    private boolean shouldWork;
    private boolean workRun = false;

    public Generator(
        Supplier<Pose2d> poseSupplier, BooleanSupplier hasCoralSupplier, Autonomous autonomous) {
      this.poseSupplier = poseSupplier;
      this.hasCoralSupplier = hasCoralSupplier;
      this.autoGen = new AutoGeneration(autonomous);
    }

    public void work() {
      shouldWork = CachedRobotState.isAutonomous() || CachedRobotState.isDisabled();

      if (CachedRobotState.isDisabled() || !CachedRobotState.isAutonomous()) {
        workRun = false;
      }

      AutoParams newParams = AutoParams.get(poseSupplier.get(), hasCoralSupplier.getAsBoolean());

      AutonChooser.setAutoComputed(autoGen.doneComputing(newParams));

      if (!shouldWork) {
        workRun = false;
        autoGen.setWorking(false);
        return;
      }

      if (workRun) {
        autoGen.setWorking(false);

        return;
      }

      autoGen.setParameters(newParams);
      autoGen.setWorking(true);

      if (!autoGen.isAlive()) {
        if (autoGen.getState() != State.NEW) autoGen = autoGen.clone();

        autoGen.start();
      }
    }

    public Command generate() {
      workRun = true;

      GeneratedAuto auto =
          autoGen.get(AutoParams.get(poseSupplier.get(), hasCoralSupplier.getAsBoolean()));

      if (auto == null) return Commands.none();

      CoralSequences.displaySequence(
          "Autonomous Sequence", auto.getSequence(), auto.getParameters().startPose());

      return auto.getCommand();
    }
  }
}
