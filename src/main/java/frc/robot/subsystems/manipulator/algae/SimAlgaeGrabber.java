package frc.robot.subsystems.manipulator.algae;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Preferences.MANIPULATOR;

public class SimAlgaeGrabber extends AlgaeGrabber {
    private boolean hasGamePiece = false;
    public Time dropTime;
    public Time intakeTime;
    public Time gripTime;
    public Time stopTime;

    public SimAlgaeGrabber() {
        this(Seconds.of(0.25), Seconds.of(0.25), MANIPULATOR.ALGAE_GRIP_CHECK_TIME, Seconds.of(0));
    }

    public SimAlgaeGrabber(Time dropTime, Time intakeTime, Time gripTime, Time stopTime) {
        this.dropTime = dropTime;
        this.intakeTime = intakeTime;
        this.gripTime = gripTime;
        this.stopTime = stopTime;
    }

    @Override
    public void expectGamePiece(boolean hasGamePiece) {
        this.hasGamePiece = hasGamePiece;
    }

    @Override
    public boolean hasGamePiece() {
        return hasGamePiece;
    }

    private Command waitTimeCommand(Time time) {
        Command command = time.isNear(Seconds.of(0), Seconds.of(0.01)) ? Commands.none() : Commands.waitTime(time);
        
        command.addRequirements(this);
        return command;
    }

    @Override
    public Command drop() {
        return waitTimeCommand(dropTime).andThen(() -> hasGamePiece = false);
    }

    @Override
    public Command intake() {
        return waitTimeCommand(intakeTime).andThen(() -> hasGamePiece = true);
    }

    @Override
    public Command checkGrip() {
        return waitTimeCommand(gripTime);
    }

    @Override
    public Command stop() {
        return waitTimeCommand(stopTime);
    }
}
