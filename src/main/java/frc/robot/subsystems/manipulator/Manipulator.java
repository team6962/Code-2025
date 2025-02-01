package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences.MANIPULATOR;

public class Manipulator extends SubsystemBase {
  public final ManipulatorPivot pivot;
  public final ManipulatorGrabber algae;
  public final ManipulatorGrabber coral;

  public Manipulator() {
    pivot = new ManipulatorPivot();

    algae =
        new ManipulatorGrabber(
            Constants.CAN.MANIPULATOR_ALGAE,
            new ManipulatorGrabber.DigitalSensor(Constants.DIO.ALGAE_BEAM_BREAK),
            MANIPULATOR.ALGAE_IN_SPEED,
            MANIPULATOR.ALGAE_OUT_SPEED,
            () -> ENABLED_SYSTEMS.MANIPULATOR);

    coral =
        new ManipulatorGrabber(
            Constants.CAN.MANIPULATOR_CORAL,
            new ManipulatorGrabber.TimeSensor(false, Seconds.of(1), Seconds.of(1)),
            MANIPULATOR.CORAL_IN_SPEED,
            MANIPULATOR.CORAL_OUT_SPEED,
            () -> ENABLED_SYSTEMS.MANIPULATOR);
  }

  public Command placeCoralL23() {
    return pivot.coralL23().andThen(coral.drop());
  }

  public Command placeCoralL4() {
    return pivot.coralL4().andThen(coral.drop());
  }

  public Command intakeCoral() {
    return pivot.intakeCoral().alongWith(coral.intake());
  }

  public Command pickupGroundAlgae() {
    return pivot.algaeGround().alongWith(algae.intake());
  }

  public Command pickupReefAlgae() {
    return pivot.algaeReef().alongWith(algae.intake());
  }

  public Command placeBargeAlgae() {
    return pivot.algaeBarge().andThen(algae.drop());
  }

  public Command placeProcessorAlgae() {
    return pivot.algaeProcessor().andThen(algae.drop());
  }
}