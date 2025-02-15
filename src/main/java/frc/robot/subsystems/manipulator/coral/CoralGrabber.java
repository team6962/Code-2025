package frc.robot.subsystems.manipulator.coral;

import java.util.Set;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;

public abstract class CoralGrabber extends SubsystemBase {
    private boolean hasGamePiece = false;

    public CoralGrabber() {
        setName("Coral Grabber");
        
        Logger.logBoolean(getName() + "/hasGamePiece", this::hasGamePiece);
        Logger.logBoolean(getName() + "/detectsGamePiece", this::detectsGamePiece);
    }

    public abstract boolean detectsGamePiece();

    public boolean hasGamePiece() {
        return hasGamePiece;
    }

    protected void setHasGamePiece(boolean hasGamePiece) {
        this.hasGamePiece = hasGamePiece;
    }

    public Command action() {
        return Commands.defer(() -> hasGamePiece() ? drop() : intake(), Set.of(this));
    }

    public abstract Command intake();
    public abstract Command drop();
    public abstract Command stop();

    public static CoralGrabber create() {
        if (!ENABLED_SYSTEMS.MANIPULATOR) return SimCoralGrabber.disabled();
        else if (RobotBase.isReal()) return new RealCoralGrabber();
        else return SimCoralGrabber.simulated();
    }
}
