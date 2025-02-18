package frc.robot.subsystems.hang;

import com.team6962.lib.utils.CommandUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DisabledHang extends SubsystemBase implements Hang {
  @Override
  public Command deploy() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command hang() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command stow() {
    return CommandUtils.noneWithRequirements(this);
  }
}
