package frc.robot.auto.pipeline;

import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.CommandUtils;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.auto.utils.AutoPaths;
import frc.robot.auto.utils.AutonomousCommands;
import java.lang.Thread.State;
import java.util.function.Supplier;

public class AutoGeneration extends SubsystemBase {
  private AutoThread currentThread;
  private AutonomousCommands autonomous;
  private Time workDelay;
  private Time workTime;
  private Supplier<AutoPaths.PlanParameters> parametersSupplier;

  public AutoGeneration(
      AutonomousCommands autonomous,
      Time sleepTime,
      Time workTime,
      Supplier<AutoPaths.PlanParameters> parametersSupplier) {
    this.autonomous = autonomous;
    this.workDelay = sleepTime;
    this.workTime = workTime;
    this.parametersSupplier = parametersSupplier;
    currentThread = new AutoThread(autonomous, sleepTime, workTime);
  }

  private void logState() {
    // Logger.log(AutoPaths.Logging.AUTO_GENERATION + "/threadExists", currentThread != null);
    // Logger.log(AutoPaths.Logging.AUTO_GENERATION + "/threadFinished", currentThread.isFinished());
  }

  @Override
  public void periodic() {
    setParameters(parametersSupplier.get(), false);
  }

  private void setParameters(AutoPaths.PlanParameters parameters, boolean forceWork) {
    // Logger.logObject(AutoPaths.Logging.AUTO_GENERATION + "/newParameters", parameters);

    if (!parameters.constraints.pathExists()
        || (!AutoThread.shouldWorkInBackground() && !forceWork)) {
      if (currentThread != null && !currentThread.isFinished()) currentThread.interrupt();

      return;
    }

    if (currentThread == null || currentThread.isFinished()) {
      // Logger.log(AutoPaths.Logging.AUTO_GENERATION + "/restartedThread", true);

      currentThread = new AutoThread(autonomous, workDelay, workTime);
      currentThread.setParameters(parameters);
      currentThread.start();
    } else if (currentThread.getState() == State.NEW) {
      currentThread.setParameters(parameters);
      currentThread.start();
    } else {
      // Logger.log(AutoPaths.Logging.AUTO_GENERATION + "/restartedThread", false);

      currentThread.setParameters(parameters);
    }

    logState();
  }

  public Command getCommand() {
    AutoPaths.PlanParameters parameters = parametersSupplier.get();

    if (!parameters.constraints.pathExists()) {
      return CommandUtils.warnWithRequirements(
          "No valid autonomous selected", autonomous.getRequirements());
    }

    setParameters(parameters, true);

    Command command = currentThread.getCommand(RobotState::isAutonomous);

    System.out.print("Auto command: ");
    System.out.println(command);

    // Logger.log(AutoPaths.Logging.AUTO_GENERATION + "/lastGetCommand", Timer.getFPGATimestamp());
    // Logger.log(AutoPaths.Logging.AUTO_GENERATION + "/commandIsNull", command == null);

    if (command == null)
      return CommandUtils.warnWithRequirements(
          "Failed to generate autonomous command", autonomous.getRequirements());
    else return command;
  }
}
