package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.pivot.IntakePivot;

public class Intake extends SubsystemBase {
  public IntakePivot pivot = IntakePivot.create();
  public IntakeWheels wheels = new IntakeWheels();
}
