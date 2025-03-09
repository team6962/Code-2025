package frc.robot.subsystems.manipulator.pivot;

import com.team6962.lib.utils.CommandUtils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class DisabledManipulatorPivot implements ManipulatorPivot {
  public Command pivotTo(Supplier<Angle> angleSupplier, Angle tolerance) {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command pivotTo(Supplier<Angle> angleSupplier) {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command hold() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command coralIntake() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command coralL1() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command coralL23() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command coralL4() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command algaeReef() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command algaeBarge() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command algaeProcessor() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command algaeGround() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command stow() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command safe(Angle tolerance) {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command safe() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command stop() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command up() {
    return CommandUtils.noneWithRequirements(this);
  }

  public Command down() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command pivotSpeed(double speed) {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Angle getAngle() {
    throw new UnsupportedOperationException("DisabledManipulatorPivot does not support getAngle()");
  }

  public void setMinMaxAngle(Angle min, Angle max) {}
}
