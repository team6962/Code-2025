package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;

public interface Elevator extends Subsystem {
  public Command moveTo(Distance height);

  public Command stop();

  public Command hold();

  public Command stow();

  public Command coralL1();

  public Command coralL2();

  public Command coralL3();

  public Command coralL4();

  public Command coralIntake();

  public Command algaeGround();

  public Command algaeL2();

  public Command algaeL3();

  public Command algaeBarge();

  public Command algaeProcessor();

  public Command rezeroAtBottom();

  public Distance getAverageHeight();

  public Distance getMaxHeight();

  public Distance getMinHeight();

  public Command move(double speed);

  public Command up();

  public Command down();

  public boolean inRange(Distance height);

  public static Elevator create() {
    if (ENABLED_SYSTEMS.isElevatorEnabled()) return new RealElevator();
    else return new DisabledElevator();
  }
}
