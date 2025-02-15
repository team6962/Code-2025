package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.AngleRange;
import frc.robot.subsystems.manipulator.algae.AlgaeGrabber;
import frc.robot.subsystems.manipulator.coral.CoralGrabber;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences.MANIPULATOR;
import java.util.Map;

public class Manipulator extends SubsystemBase {
  public final ManipulatorPivot pivot;
  public final AlgaeGrabber algae;
  public final CoralGrabber coral;

  public Manipulator() {
    pivot = new ManipulatorPivot();

    algae = AlgaeGrabber.create();
    coral = CoralGrabber.create();
  }

  public void setPivotAnglesBasedOnHeight(Distance elevatorHeight) {
    Map.Entry<Double, AngleRange> entry =
        Constants.ELEVATOR.HEIGHT_TO_ANGLE_MAP.floorEntry(elevatorHeight.in(Meters));
    if (entry != null) {
      AngleRange angleRange = entry.getValue();
      pivot.setMinMaxAngle(angleRange.getMinAngle(), angleRange.getMaxAngle());
      //   System.out.println("Set min and max angles for height " + elevatorHeight + " to " +
      // angleRange);
      // } else {
      //   System.out.println("No angle range found for height " + elevatorHeight);
    }
  }

  public Command placeCoralL1() {
    return pivot.coralL1().andThen(coral.drop());
  }

  public Command placeCoralL23() {
    return pivot.coralL23().andThen(coral.drop());
  }

  public Command placeCoralL4() {
    return pivot.coralL4().andThen(coral.drop());
  }

  public Command intakeCoral() {
    return pivot.coralIntake().alongWith(coral.intake());
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
        pivot.safe(), /* intakeCoral(), placeCoralL23(),*/ pickupGroundAlgae(), placeProcessorAlgae(), pivot.safe());
  }
}
