package frc.robot.auto.pipeline;

import java.lang.Thread.State;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.utils.AutoPaths;
import frc.robot.auto.utils.AutonomousCommands;

public class AutoGeneration {
    private AutoThread currentThread;
    private AutonomousCommands autonomous;

    public AutoGeneration(AutonomousCommands autonomous) {
        this.autonomous = autonomous;
        currentThread = new AutoThread(autonomous);
    }

    private void logState() {
        Logger.log(AutoPaths.Logging.AUTO_GENERATION + "/threadExists", currentThread != null);
        Logger.log(AutoPaths.Logging.AUTO_GENERATION + "/threadFinished", currentThread.isFinished());
    }

    public void setParameters(AutoPaths.PlanParameters parameters) {
        Logger.logObject(AutoPaths.Logging.AUTO_GENERATION + "/newParameters", parameters);

        if (!parameters.constraints.pathExists() || !AutoThread.shouldWorkInBackground()) {
            if (currentThread != null && !currentThread.isFinished()) currentThread.interrupt();

            return;
        }

        if (currentThread == null || currentThread.isFinished()) {
            Logger.log(AutoPaths.Logging.AUTO_GENERATION + "/restartedThread", true);

            currentThread = new AutoThread(autonomous);
            currentThread.setParameters(parameters);
            currentThread.start();
        } else if (currentThread.getState() == State.NEW) {
            currentThread.setParameters(parameters);
            currentThread.start();
        } else {
            Logger.log(AutoPaths.Logging.AUTO_GENERATION + "/restartedThread", false);

            currentThread.setParameters(parameters);
        }

        logState();
    }

    public Command getCommand() {
        Command command = currentThread.getCommand(RobotState::isAutonomous);

        System.out.print("Auto command: ");
        System.out.println(command);

        Logger.log(AutoPaths.Logging.AUTO_GENERATION + "/lastGetCommand", Timer.getFPGATimestamp());
        Logger.log(AutoPaths.Logging.AUTO_GENERATION + "/commandIsNull", command == null);

        if (command == null) return Commands.none();
        else return command;
    }
}
