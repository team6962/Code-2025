package com.team6962.lib.utils;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class CommandUtils {
  private CommandUtils() {}

  public static Command noneWithRequirements(Subsystem... requirements) {
    Command command = Commands.none();

    for (Subsystem requirement : requirements) {
      command.addRequirements(requirement);
    }

    return command;
  }

  public static Command selectByMode(Command realCommand, Command simCommand) {
    // return simCommand;
    return RobotBase.isReal() ? realCommand : simCommand;
  }

  public static Command printAndWait(String message, double seconds) {
    // return Commands.parallel(Commands.print(message), Commands.waitSeconds(seconds));

    return Commands.print(message);
  }

  public static Command printInSimulation(String message) {
    return RobotBase.isSimulation() ? Commands.print(message) : Commands.none();
  }

  public static Command withRequirements(Command command, Subsystem... requirements) {
    command.addRequirements(requirements);

    return command;
  }

  public static Command waitFor(Command otherCommand) {
    return Commands.waitUntil(() -> otherCommand.isFinished());
  }
}
