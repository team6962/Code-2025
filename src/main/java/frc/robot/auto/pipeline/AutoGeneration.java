package frc.robot.auto.pipeline;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.utils.AutoPaths;
import frc.robot.auto.utils.AutonomousCommands;

public class AutoGeneration {
    private AutoThread currentThread;
    private AutonomousCommands autonomous;

    public AutoGeneration(AutonomousCommands autonomous) {
        this.autonomous = autonomous;
        currentThread = null;
    }

    public void setParameters(AutoPaths.PlanParameters parameters) {
        if (currentThread == null || currentThread.isFinished()) {
            currentThread = new AutoThread(autonomous);
            currentThread.setParameters(parameters);
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
