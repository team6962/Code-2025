package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private IntakePivot pivot;
    private IntakeWheels wheels;

    public Command intake() {
        return Commands.parallel(
            pivot.raise().repeatedly(),
            wheels.intake(),
            Commands.waitSeconds(1.0)
        );
    }

    public Command drop() {
        return Commands.sequence(
            pivot.lower(),
            wheels.drop(),
            Commands.waitSeconds(1.0)
        );
    }
}
