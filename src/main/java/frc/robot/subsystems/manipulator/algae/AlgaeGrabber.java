package frc.robot.subsystems.manipulator.algae;

import java.util.Set;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;

public abstract class AlgaeGrabber extends SubsystemBase {
    private boolean hasGamePiece = false;

    public AlgaeGrabber() {
        setName("Algae Grabber");

        Logger.logBoolean(getName() + "/hasGamePiece", this::hasGamePiece);
    }

    public void expectGamePiece(boolean hasGamePiece) {
        this.hasGamePiece = hasGamePiece;
    }

    public boolean hasGamePiece() {
        return hasGamePiece;
    }

    public abstract Command checkGrip();
    public abstract Command intake();
    public abstract Command drop();
    public abstract Command stop();

    public Command action() {
        return Commands.defer(() -> hasGamePiece() ? drop() : intake(), Set.of(this));
    }

    public static AlgaeGrabber create() {
        if (!ENABLED_SYSTEMS.MANIPULATOR) return SimAlgaeGrabber.disabled();
        else if (RobotBase.isReal()) return new RealAlgaeGrabber();
        else return SimAlgaeGrabber.simulated();
    }
}
