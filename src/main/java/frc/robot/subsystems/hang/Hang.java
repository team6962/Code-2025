package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj2.command.Command;

public interface Hang {
  public Command deploy();

  public Command hang();

  public Command stow();

  public static Hang create() {
    return new RealHang();
  }
}
