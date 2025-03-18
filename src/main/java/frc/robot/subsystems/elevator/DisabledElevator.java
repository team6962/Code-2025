package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.team6962.lib.utils.CommandUtils;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DisabledElevator extends SubsystemBase implements Elevator {
  public Command setHeight(Distance height) {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command stop() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command hold() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command stow() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command coralL1() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command coralL2() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command coralL3() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command coralL4() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command coralIntake() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command algaeGround() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command algaeL2() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command algaeL3() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command algaeBarge() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command algaeProcessor() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command rezeroAtBottom() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Distance getAverageHeight() {
    return Meters.of(0.0);
  }

  public Command up() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command down() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Distance getMaxHeight() {
    return Meters.of(Double.POSITIVE_INFINITY);
  }

  public Distance getMinHeight() {
    return Meters.of(Double.NEGATIVE_INFINITY);
  }

  public boolean inRange(Distance height) {
    return true;
  }
}
