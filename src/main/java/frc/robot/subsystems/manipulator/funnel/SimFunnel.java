package frc.robot.subsystems.manipulator.funnel;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.grabber.Grabber;

public class SimFunnel extends Funnel {
  private final Time intakeTime;
  private final Time stopTime;

  public SimFunnel(Time intakeTime, Time stopTime) {
    this.intakeTime = intakeTime;
    this.stopTime = stopTime;
  }

  public static SimFunnel simulated() {
    return new SimFunnel(Seconds.of(0.25), Seconds.of(0));
  }

  public static SimFunnel disabled() {
    return new SimFunnel(Seconds.of(0), Seconds.of(0));
  }

  private Command wait(Time time) {
    Command command =
        time.isNear(Seconds.of(0), Seconds.of(0.01)) ? Commands.none() : Commands.waitTime(time);

    command.addRequirements(this);
    return command;
  }

  @Override
  public Command intake(Grabber coral) {
    return wait(intakeTime);
  }

  @Override
  public Command stop() {
    return wait(stopTime);
  }
}
