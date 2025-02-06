package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences.ELEVATOR;
import frc.robot.util.hardware.MotionControl.DualLinearController;

public class Elevator extends DualLinearController {
  public Elevator() {
    super(
        CAN.ELEVATOR_LEFT,
        CAN.ELEVATOR_RIGHT,
        DIO.ELEVATOR_ENCODER,
        Constants.ELEVATOR.ENCODER_OFFSET.in(Rotations), // CHANGE THIS
        Constants.ELEVATOR.PROFILE.kP,
        Constants.ELEVATOR.PROFILE.kS,
        Constants.ELEVATOR.GEARING,
        Constants.ELEVATOR.CYCLE_HEIGHT,
        ELEVATOR.MIN_HEIGHT,
        ELEVATOR.MAX_HEIGHT,
        Inches.of(0.5));


    // setDefaultCommand(Commands.run(this::stopMotors, this));
  }

  public Command setHeightCommand(Distance height) {
    return this.run(() -> setTargetHeightAndRun(height)).until(this::doneMoving);
  }

  public Command up() {
    return Commands.runEnd(this::moveUp, this::stopMotors);
  }

  public Command down() {
    return Commands.runEnd(this::moveDown, this::stopMotors);
  }

  public Command coralL1() {
    return setHeightCommand(ELEVATOR.CORAL.L1_HEIGHT);
  }

  public Command coralL2() {
    return setHeightCommand(ELEVATOR.CORAL.L2_HEIGHT);
  }

  public Command coralL3() {
    return setHeightCommand(ELEVATOR.CORAL.L3_HEIGHT);
  }

  public Command coralL4() {
    return setHeightCommand(ELEVATOR.CORAL.L4_HEIGHT);
  }

  public Command coralIntake() {
    return setHeightCommand(ELEVATOR.CORAL.INTAKE_HEIGHT);
  }

  public Command algaeGround() {
    return setHeightCommand(ELEVATOR.ALGAE.GROUND_HEIGHT);
  }

  public Command algaeL2() {
    return setHeightCommand(ELEVATOR.ALGAE.L2_HEIGHT);
  }

  public Command algaeL3() {
    return setHeightCommand(ELEVATOR.ALGAE.L3_HEIGHT);
  }

  public Command algaeBarge() {
    return setHeightCommand(ELEVATOR.ALGAE.BARGE_HEIGHT);
  }

  public Command stow() {
    return setHeightCommand(ELEVATOR.STOW_HEIGHT);
  }

  public void setTargetHeightAndRun(Distance height) {
    System.out.println("MOVING UP =====");
    setTargetHeight(height);
    run();
  }

  @Override
  public void run() {
    if (!ENABLED_SYSTEMS.ELEVATOR) return;
    super.run();
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ELEVATOR) stopMotors();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
