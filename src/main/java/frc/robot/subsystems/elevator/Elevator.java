package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Constants.ELEVATOR;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;

public interface Elevator extends Subsystem {
  public Command moveTo(Distance height);

  public Command stop();

  public Command hold();

  public default Command stow() {
    return moveTo(ELEVATOR.STOW_HEIGHT);
  }

  public default Command coralL1() {
    return moveTo(ELEVATOR.CORAL.L1_HEIGHT);
  }

  public default Command coralL2() {
    return moveTo(ELEVATOR.CORAL.L2_HEIGHT);
  }

  public default Command coralL3() {
    return moveTo(ELEVATOR.CORAL.L3_HEIGHT);
  }

  public default Command coralL4() {
    return moveTo(ELEVATOR.CORAL.L4_HEIGHT);
  }

  public default Command coralIntake() {
    return moveTo(ELEVATOR.CORAL.INTAKE_HEIGHT);
  }

  public default Command algaeGround() {
    return moveTo(ELEVATOR.ALGAE.GROUND_HEIGHT);
  }

  public default Command algaeL2() {
    return moveTo(ELEVATOR.ALGAE.L2_HEIGHT);
  }

  public default Command algaeL3() {
    return moveTo(ELEVATOR.ALGAE.L3_HEIGHT);
  }

  public default Command algaeBarge() {
    return moveTo(ELEVATOR.ALGAE.BARGE_HEIGHT);
  }

  public default Command algaeProcessor() {
    return moveTo(ELEVATOR.ALGAE.PROCESSOR_HEIGHT);
  }

  public Distance getPosition();

  public Distance getMaxPosition();

  public Distance getMinPosition();

  public Command moveDutyCycle(double speed);

  public default Command up() {
      return this.moveDutyCycle(ELEVATOR.FINE_CONTROL_DUTY_CYCLE);
  }

  public default Command down() {
      return this.moveDutyCycle(-ELEVATOR.FINE_CONTROL_DUTY_CYCLE);
  }

  public boolean atPosition(Distance height);

  public static Elevator create() {
    if (ENABLED_SYSTEMS.isElevatorEnabled()) return new RealElevator();
    else return new DisabledElevator();
  }
}
