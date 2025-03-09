package frc.robot.subsystems.intake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.FUNNEL;
import frc.robot.subsystems.funnel.FunnelBeamBreak;
import frc.robot.subsystems.funnel.FunnelMotor;

/**
 * A subsystem that can intake coral using the funnel. This is the version 2
 * robot's implementation of {@link CoralIntake}.
 */
public class FunnelIntake implements CoralIntake {
    private FunnelMotor motor;
    private FunnelBeamBreak beamBreak;
    private boolean unsafe = false;

    public FunnelIntake() {
        if (ENABLED_SYSTEMS.FUNNEL) {
            motor = FunnelMotor.get();
            beamBreak = FunnelBeamBreak.get(motor);
        }
    }

    @Override
    public Command intake() {
        if (!ENABLED_SYSTEMS.FUNNEL) {
            return Commands.none();
        }

        return Commands.runOnce(() -> beamBreak.expectGamePiece())
            .andThen(Commands.waitUntil(beamBreak::detectsGamePiece))
            .andThen(Commands.runOnce(() -> {
                unsafe = true;
                motor.intake().withTimeout(FUNNEL.INTAKE_TIME).finallyDo(() -> unsafe = false).schedule();
            }));
    }

    @Override
    public Set<Subsystem> getSubsystems() {
        if (!ENABLED_SYSTEMS.FUNNEL) {
            return Set.of();
        }

        return Set.of(motor);
    }

    @Override
    public boolean isUnsafeToMoveManipulator() {
        return unsafe;
    }

    @Override
    public Command fineControl(double power) {
        if (!ENABLED_SYSTEMS.FUNNEL) {
            return Commands.none();
        }

        return motor.fineControl(power);
    }
}
