package com.team6962.lib.utils;

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
}
