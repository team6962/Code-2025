package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.manipulator.algae.AlgaeGrabber;
import frc.robot.subsystems.manipulator.coral.CoralGrabber;
import frc.robot.subsystems.manipulator.pivot.ManipulatorPivot;
import frc.robot.subsystems.manipulator.funnel.Funnel;

public class Manipulator extends SubsystemBase {
  public final ManipulatorPivot pivot;
  public final AlgaeGrabber algae;
  public final CoralGrabber coral;
  public final Funnel funnel;

  public Manipulator() {
    pivot = ManipulatorPivot.create();
    algae = AlgaeGrabber.create();
    coral = CoralGrabber.create();
    funnel = Funnel.create();
  }

  public Command placeCoralL1() {
    return pivot.coralL1();
  }

  public Command placeCoralL23() {
    return pivot.coralL23();
  }

  public Command placeCoralL4() {
    return pivot.coralL4();
  }

  public Command intakeCoral() {
    return pivot.coralIntake().alongWith(coral.intake().alongWith(funnel.intake(coral)));
  }

  public Command pickupGroundAlgae() {
    return pivot.algaeGround().alongWith(algae.intake());
  }

  public Command pickupReefAlgae() {
    return pivot.algaeReef().alongWith(algae.intake());
  }

  public Command dropReefAlgae() {
    return pivot.algaeReef().alongWith(algae.drop());
  }

  public Command placeBargeAlgae() {
    return pivot.algaeBarge().andThen(algae.drop());
  }

  public Command placeProcessorAlgae() {
    return pivot.algaeProcessor();
  }

  public Command stow() {
    return pivot.stow();
  }

  public Command stop() {
    return pivot.stop().alongWith(coral.stop(), algae.stop());
  }

  public Command test() {
    return Commands.sequence(
        pivot.safe(), /* intakeCoral(), placeCoralL23(),*/
        pickupGroundAlgae(),
        placeProcessorAlgae(),
        pivot.safe());
  }
}
