package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.autonomous.Autonomous;

public class AutoGeneration {
    private AutoThread currentThread;
    private Autonomous autonomous;

    public AutoGeneration(Autonomous autonomous) {
        this.autonomous = autonomous;
        currentThread = null;
    }

    public void setParameters(AutoPaths.PlanParameters parameters) {
        if (currentThread == null || currentThread.isFinished()) {
            currentThread = new AutoThread(autonomous, parameters);
            currentThread.start();
        } else {
            currentThread.setParameters(parameters);
        }
    }

    public Command getCommand() {
        Command command = currentThread.getCommand();

        if (command == null) return Commands.none();
        else return command;
    }
}
