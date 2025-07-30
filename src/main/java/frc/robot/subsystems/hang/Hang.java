package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ENABLED_SYSTEMS;

public interface Hang {
  public Command deploy();

  public Command hang();

  public Command stow();

  public static Hang create() {
    if (ENABLED_SYSTEMS.isHangEnabled()) return new RealHang();
    else return new DisabledHang();
  }
}
