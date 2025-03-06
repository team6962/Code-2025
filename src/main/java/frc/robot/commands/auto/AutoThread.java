package frc.robot.commands.auto;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autonomous.Autonomous;

public class AutoThread extends Thread {
    private BestCommandBuilder commandBuilder;
    private AutoPaths.PlanParameters parameters;
    private Autonomous autonomous;
    private AtomicBoolean shouldWork;

    public AutoThread(Autonomous autonomous, AutoPaths.PlanParameters parameters) {
        this.autonomous = autonomous;
        this.commandBuilder = null;
    }

    public void setParameters(AutoPaths.PlanParameters parameters) {
        if (parameters == null) {
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
            BestCommandBuilder builder = new BestCommandBuilder(autonomous, parameters.constraints, parameters.startPose);

            synchronized (this) {
                this.parameters = parameters;
                commandBuilder = builder;
            }
        }
    }

    @Override
    public void run() {
        while (true) {
            if (!shouldWork.get()) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    return;
                }

                continue;
            }

            BestCommandBuilder builder;

            synchronized (this) {
                builder = commandBuilder;
            }

            if (builder != null) builder.work();
        }
    }

    public Command getCommand() {
        if (commandBuilder == null) return null;

        Command command;

        synchronized (this) {
            command = commandBuilder.getCommand();
        }

        if (command != null) return command;

        interrupt();

        synchronized (this) {
            return commandBuilder.getCommand();
        }
    }

    public boolean isFinished() {
        return getState() != State.TERMINATED;
    }
}
