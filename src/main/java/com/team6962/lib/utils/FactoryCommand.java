package com.team6962.lib.utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class FactoryCommand extends Command {
    private Supplier<Command> factory;
    private Command command;
    private boolean runWhenDisabled = false;
    private InterruptionBehavior interruptBehavior = InterruptionBehavior.kCancelSelf;

    public FactoryCommand(Supplier<Command> factory) {
        this.factory = factory;
    }

    public FactoryCommand(Supplier<Command> factory, boolean runWhenDisabled) {
        this.factory = factory;
        this.runWhenDisabled = runWhenDisabled;
    }

    public FactoryCommand(Supplier<Command> factory, InterruptionBehavior interruptBehavior) {
        this.factory = factory;
        this.interruptBehavior = interruptBehavior;
    }

    public FactoryCommand(Supplier<Command> factory, boolean runWhenDisabled, InterruptionBehavior interruptBehavior) {
        this.factory = factory;
        this.runWhenDisabled = runWhenDisabled;
        this.interruptBehavior = interruptBehavior;
    }

    @Override
    public void initialize() {
        command = factory.get();

        CommandScheduler.getInstance().registerComposedCommands(command);

        addRequirements(command.getRequirements());

        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        return runWhenDisabled;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return interruptBehavior;
    }
}
