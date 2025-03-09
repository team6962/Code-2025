package frc.robot.subsystems.intake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DisabledIntake implements CoralIntake {
    @Override
    public Command intake() {
        return Commands.none();
    }

    @Override
    public boolean isUnsafeToMoveManipulator() {
        return false;
    }

    @Override
    public Set<Subsystem> getSubsystems() {
        return Set.of();
    }

    @Override
    public Command fineControl(double power) {
        return Commands.none();
    }
}
