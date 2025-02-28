package frc.robot.subsystems.manipulator.coral;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SimCoralGrabber extends CoralGrabber {
  private final Time intakeTime;
  private final Time dropTime;
  private final Time stopTime;
  private boolean hasGamePiece = true;

  public SimCoralGrabber(Time intakeTime, Time dropTime, Time stopTime) {
    this.intakeTime = intakeTime;
    this.dropTime = dropTime;
    this.stopTime = stopTime;
  }

  public static SimCoralGrabber simulated() {
    return new SimCoralGrabber(Seconds.of(0.25), Seconds.of(0.25), Seconds.of(0));
  }

  public static SimCoralGrabber disabled() {
    return new SimCoralGrabber(Seconds.of(0), Seconds.of(0), Seconds.of(0));
  }

  private Command wait(Time time) {
    Command command =
        time.isNear(Seconds.of(0), Seconds.of(0.01)) ? Commands.none() : Commands.waitTime(time);

    command.addRequirements(this);
    return command;
  }

  @Override
  public boolean hasGamePiece() {
      return hasGamePiece;
  }

  public void expectGamePiece(boolean value) {
    hasGamePiece = value;
  }

  @Override
  public Command intake() {
    return wait(intakeTime).andThen(runOnce(() -> expectGamePiece(true)));
  }

  @Override
  public Command drop() {
    return wait(dropTime).andThen(runOnce(() -> expectGamePiece(false)));
  }

  @Override
  public Command stop() {
    return wait(stopTime);
  }
}
