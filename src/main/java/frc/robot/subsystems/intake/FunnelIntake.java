package frc.robot.subsystems.intake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Constants.FUNNEL;
import frc.robot.subsystems.funnel.FunnelBeamBreak;
import frc.robot.subsystems.funnel.FunnelMotor;

/**
 * A subsystem that can intake coral using the funnel. This is the version 2
 * robot's implementation of {@link CoralIntake}.
 */
public class FunnelIntake implements CoralIntake {
    private FunnelMotor motor = FunnelMotor.get();
    private FunnelBeamBreak beamBreak = FunnelBeamBreak.get(motor);
    private boolean unsafe = false;

    @Override
    public Command intake() {
        return Commands.runOnce(() -> beamBreak.expectGamePiece())
            .andThen(Commands.waitUntil(beamBreak::detectsGamePiece))
            .andThen(Commands.runOnce(() -> {
                unsafe = true;
                motor.intake().withTimeout(FUNNEL.INTAKE_TIME).finallyDo(() -> unsafe = false).schedule();
            }));
    }

    @Override
    public Set<Subsystem> getSubsystems() {
        return Set.of(motor);
    }

    @Override
    public boolean isUnsafeToMoveManipulator() {
        return unsafe;
    }

    @Override
    public Command fineControl(double power) {
        return motor.fineControl(power);
    }
}
