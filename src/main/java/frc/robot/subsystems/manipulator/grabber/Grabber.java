package frc.robot.subsystems.manipulator.grabber;

import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ENABLED_SYSTEMS;
import java.util.Set;

public abstract class Grabber extends SubsystemBase {
  private boolean hasAlgae = false;

  public Grabber() {
    setName("Grabber");

    Logger.logBoolean(getName() + "/hasAlgae", this::hasAlgae);
    Logger.logBoolean(getName() + "/hasCoral", this::hasCoral);
  }

  public abstract boolean hasCoral();

  public void expectAlgae(boolean hasAlgae) {
    this.hasAlgae = hasAlgae;
  }

  public boolean hasAlgae() {
    return hasAlgae;
  }

  public Command coralMagicButton() {
    return Commands.defer(() -> hasCoral() ? dropCoral() : intakeCoral(), Set.of(this));
  }

  public Command algaeMagicButton() {
    return Commands.defer(() -> hasAlgae() ? dropAlgae() : intakeAlgae(), Set.of(this));
  }

  public abstract Command hold();

  public abstract Command intakeCoral();

  public abstract Command dropCoral();

  public abstract Command adjustCoral();

  public abstract Command intakeAlgae();

  public abstract Command dropAlgae();

  public abstract Command holdAlgae();

  public abstract Command stop();

  public abstract Command repositionCoral();

  public boolean isCoralClear() {
    return false;
  }

  public Command forwards() {
    return Commands.none();
  }

  public Command backwards() {
    return Commands.none();
  }

  public static Grabber create() {
    if (!ENABLED_SYSTEMS.isManipulatorEnabled()) return SimGrabber.disabled();
    else if (RobotBase.isReal()) return new RealGrabber();
    else return SimGrabber.simulated();
  }
}
