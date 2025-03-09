package frc.robot.subsystems.manipulator.coral;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;

public abstract class CoralGrabber extends SubsystemBase {
  public CoralGrabber() {
    setName("Coral Grabber");

    Logger.logBoolean(getName() + "/hasGamePiece", this::hasGamePiece);
  }

  public abstract boolean hasGamePiece();

  public abstract Command intake();

  public abstract Command drop();

  public abstract Command stop();

  public Command fineControl(double speed) {
    return run(() -> {});
  }

  public static CoralGrabber create() {
    if (!ENABLED_SYSTEMS.MANIPULATOR) return SimCoralGrabber.disabled();
    else if (RobotBase.isReal()) return new RealCoralGrabber();
    else return SimCoralGrabber.simulated();
  }
}
