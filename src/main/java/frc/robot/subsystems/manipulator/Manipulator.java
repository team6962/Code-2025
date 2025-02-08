package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences.MANIPULATOR;
import frc.robot.Constants.Constants.AngleRange;
public class Manipulator extends SubsystemBase {
  public final ManipulatorPivot pivot;
  public final ManipulatorGrabber algae;
  public final ManipulatorGrabber coral;

  public Manipulator() {
    pivot = new ManipulatorPivot();

    algae =
        new ManipulatorGrabber(
            CAN.MANIPULATOR_ALGAE,
            new ManipulatorGrabber.DigitalSensor(Constants.DIO.ALGAE_BEAM_BREAK),
            MANIPULATOR.ALGAE_IN_SPEED,
            MANIPULATOR.ALGAE_OUT_SPEED,
            () -> ENABLED_SYSTEMS.MANIPULATOR);

    coral =
        new ManipulatorGrabber(
            CAN.MANIPULATOR_CORAL,
            new ManipulatorGrabber.TimeSensor(false, Seconds.of(0.3), Seconds.of(0.3)),
            MANIPULATOR.CORAL_IN_SPEED,
            MANIPULATOR.CORAL_OUT_SPEED,
            () -> ENABLED_SYSTEMS.MANIPULATOR);
  }

  public void setPivotAnglesBasedOnHeight(Distance elevatorHeight) {
    Map.Entry<Double, AngleRange> entry = Constants.ELEVATOR.HEIGHT_TO_ANGLE_MAP.floorEntry(elevatorHeight.in(Meters));
    if (entry != null) {
      AngleRange angleRange = entry.getValue();
      pivot.setMinMaxAngle(angleRange.getMinAngle(), angleRange.getMaxAngle());
    //   System.out.println("Set min and max angles for height " + elevatorHeight + " to " + angleRange);
    // } else {
    //   System.out.println("No angle range found for height " + elevatorHeight);
    }
  }

  public void calculateDynamicLimit(Distance elevatorHeight){
    
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

  public Command stow() {
    return pivot.stow();
  }

  public Command stop() {
    return pivot.stop().alongWith(coral.stop(), algae.stop());
  }
  
  public Command test() {
    return Commands.sequence(
      intakeCoral(),
      placeCoralL23(),
      pickupGroundAlgae(),
      placeProcessorAlgae(),
      stow()
    );
  }
}