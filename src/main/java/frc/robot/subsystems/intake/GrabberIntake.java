package frc.robot.subsystems.intake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.manipulator.coral.CoralGrabber;

/**
 * A subsystem that can intake coral using the manipulator. This is the version 1
 * robot's implementation of {@link CoralIntake}.
 */
public class GrabberIntake implements CoralIntake {
    private CoralGrabber grabber;

    public GrabberIntake(CoralGrabber grabber) {
        this.grabber = grabber;
    }

    @Override
    public Command intake() {
        return grabber.intake();
    }

    @Override
    public Set<Subsystem> getSubsystems() {
        return Set.of(grabber);
    }

    @Override
    public boolean isUnsafeToMoveManipulator() {
        return false;
    }

    @Override
    public Command fineControl(double power) {
        return grabber.fineControl(power);
    }
}
