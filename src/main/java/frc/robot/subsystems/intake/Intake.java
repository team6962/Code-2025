package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public IntakePivot pivot = new IntakePivot();
  public IntakeWheels wheels = new IntakeWheels();
}
