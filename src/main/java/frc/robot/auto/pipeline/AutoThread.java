package frc.robot.auto.pipeline;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.utils.AutoPaths;
import frc.robot.auto.utils.AutonomousCommands;

public class AutoThread extends Thread {
    private CommandBuilder commandBuilder;
    private AutoPaths.PlanParameters parameters;

    private static final Time WORK_DELAY = Milliseconds.of(20);

    public AutoThread(AutonomousCommands autonomous) {
        this.commandBuilder = new CommandBuilder(autonomous);
    }

    private boolean shouldWorkInBackground() {
        return RobotState.isDisabled();
    }

    public void setParameters(AutoPaths.PlanParameters parameters) {
        if (parameters == null || !parameters.constraints.pathExists()) {
            synchronized (this) {
                this.parameters = null;
                commandBuilder = null;
            }
        }

        boolean needsReplacement;

        synchronized (this) {
            needsReplacement = this.parameters == null || !this.parameters.matches(parameters);
        }

        if (needsReplacement) {
            synchronized (this) {
                this.parameters = parameters;
                commandBuilder.start(parameters);
            }
        }
    }

    @Override
    public void run() {
        while (true) {
            if (!shouldWorkInBackground()) {
                try {
                    Thread.sleep((long) WORK_DELAY.in(Milliseconds));
                } catch (InterruptedException e) {
                    return;
                }

                continue;
            }

            CommandBuilder builder;

            synchronized (this) {
                builder = commandBuilder;
            }

            if (builder != null) builder.work();
        }
    }

    public Command getCommand() {
        if (commandBuilder == null) return null;

        boolean done;

        synchronized (this) {
            done = commandBuilder.isDone();
        }

        if (!done) {
            synchronized (this) {
                return commandBuilder.getPathCommand();
            }
        }

        interrupt();

        synchronized (this) {
            while (!commandBuilder.isDone()) {
                commandBuilder.work();
            }

            return commandBuilder.getPathCommand();
        }
    }

    public boolean isFinished() {
        return getState() != State.TERMINATED;
    }
}
