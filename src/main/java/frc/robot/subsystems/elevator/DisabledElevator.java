package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.team6962.lib.utils.CommandUtils;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DisabledElevator extends SubsystemBase implements Elevator {
  @Override
  public Command moveTo(Distance height) {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command stop() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command hold() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command stow() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command coralL1() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command coralL2() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command coralL3() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command coralL4() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command coralIntake() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command algaeGround() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command algaeL2() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command algaeL3() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command algaeBarge() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command algaeProcessor() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Distance getPosition() {
    return Meters.of(0.0);
  }

  @Override
  public Command moveDutyCycle(double speed) {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command up() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command down() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Distance getMaxPosition() {
    return Meters.of(Double.POSITIVE_INFINITY);
  }

  @Override
  public Distance getMinPosition() {
    return Meters.of(Double.NEGATIVE_INFINITY);
  }

  @Override
  public boolean atPosition(Distance height) {
    return true;
  }
}
