package frc.robot.auto.pipeline;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.utils.AutoPaths;
import frc.robot.auto.utils.AutonomousCommands;
import java.util.function.BooleanSupplier;

public class AutoThread extends Thread {
  private CommandBuilder commandBuilder;
  private AutoPaths.PlanParameters parameters;
  private final long sleepTimeMilliseconds;
  private final double workTimeSeconds;
  private double lastPauseTimestampSeconds;

  public AutoThread(AutonomousCommands autonomous, Time sleepTime, Time workTime) {
    this.commandBuilder = new CommandBuilder(autonomous);
    this.sleepTimeMilliseconds = (long) sleepTime.in(Milliseconds);
    this.workTimeSeconds = workTime.in(Seconds);
    lastPauseTimestampSeconds = Timer.getFPGATimestamp();

    // Logger.logObject(AutoPaths.Logging.AUTO_THREAD + "/parameters", "null");
    // Logger.log(AutoPaths.Logging.AUTO_THREAD + "/commandBuilderExists", commandBuilder != null);

    logWorkState(WorkState.NEW);
  }

  public static boolean shouldWorkInBackground() {
    return RobotState.isDisabled();
  }

  public void setParameters(AutoPaths.PlanParameters parameters) {
    if (parameters == null || !parameters.constraints.pathExists()) {
      synchronized (this) {
        this.parameters = null;
      }

      // Logger.log(AutoPaths.Logging.AUTO_THREAD + "/parametersReplaced", true);
      // Logger.logObject(AutoPaths.Logging.AUTO_THREAD + "/parameters", parameters);
      // Logger.log(AutoPaths.Logging.AUTO_THREAD + "/commandBuilderExists", commandBuilder != null);

      return;
    }

    boolean needsReplacement;

    synchronized (this) {
      needsReplacement = this.parameters == null || !this.parameters.matches(parameters);
    }

    // Logger.log(AutoPaths.Logging.AUTO_THREAD + "/parametersReplaced", needsReplacement);

    // System.out.println("Old parameters: " + this.parameters);
    // System.out.println("New parameters: " + parameters);
    // if (this.parameters != null) System.out.println("Matches: " +
    // this.parameters.matches(parameters));
    // System.out.println("Needs replacement: " + needsReplacement);

    if (needsReplacement) {
      synchronized (this) {
        this.parameters = parameters;
        commandBuilder.start(parameters);
      }
    }
  }

  @Override
  public void run() {
    System.out.println("Starting auto thread...");

    while (true) {
      // Logger.log(AutoPaths.Logging.AUTO_THREAD + "/loopTimestamp", Timer.getFPGATimestamp());

      CommandBuilder builder;

      synchronized (this) {
        builder = commandBuilder;
      }

      boolean cantWork = !shouldWorkInBackground()
        || Timer.getFPGATimestamp() - lastPauseTimestampSeconds > workTimeSeconds
        || builder == null;

      if (cantWork || builder.isDone()) {
        logWorkState(WorkState.SLEEPING);

        try {
          Thread.sleep((long) sleepTimeMilliseconds);
        } catch (InterruptedException e) {
          // Logger.log(AutoPaths.Logging.AUTO_THREAD + "/interruptTime", Timer.getFPGATimestamp());

          logWorkState(WorkState.DEAD);

          System.out.println("Auto thread stopped");

          return;
        }

        lastPauseTimestampSeconds = Timer.getFPGATimestamp();

        continue;
      }

      logWorkState(WorkState.WORKING);

      builder.work();
    }
  }

  public Command getCommand() {
    return getCommand(() -> true);
  }

  public Command getCommand(BooleanSupplier shouldContinueComputing) {
    // Logger.log(AutoPaths.Logging.AUTO_THREAD + "/command/updateTime", Timer.getFPGATimestamp());
    // Logger.log(
    //     AutoPaths.Logging.AUTO_THREAD + "/command/hasCommandBuilder", commandBuilder != null);

    if (commandBuilder == null) return null;

    boolean done;

    synchronized (this) {
      done = commandBuilder.isDone();
    }

    // Logger.log(AutoPaths.Logging.AUTO_THREAD + "/command/isDone", done);

    if (done) {
      synchronized (this) {
        return commandBuilder.getPathCommand();
      }
    }

    // Logger.log(AutoPaths.Logging.AUTO_THREAD + "/command/interrupting", true);

    interrupt();

    synchronized (this) {
      boolean worked = !commandBuilder.isDone();

      while (!commandBuilder.isDone()) {
        commandBuilder.work();

        if (!shouldContinueComputing.getAsBoolean()) return Commands.none();
      }

      // Logger.log(AutoPaths.Logging.AUTO_THREAD + "/command/worked", worked);

      return commandBuilder.getPathCommand();
    }
  }

  public boolean isFinished() {
    return getState() == State.TERMINATED;
  }

  private static WorkState state;

  private static void logWorkState(WorkState state) {
    if (AutoThread.state != state) {
      AutoThread.state = state;

      // Logger.log(AutoPaths.Logging.AUTO_THREAD + "/state", state.toString());
    }
  }

  public static enum WorkState {
    NEW,
    SLEEPING,
    WORKING,
    DEAD
  }
}
