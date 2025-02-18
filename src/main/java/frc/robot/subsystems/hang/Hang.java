package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;

public interface Hang {
  public Command deploy();

  public Command hang();

  public Command stow();

  public static Hang create() {
    if (ENABLED_SYSTEMS.HANG) return new RealHang();
    else return new DisabledHang();
  }
}
