package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
          0124124, // CHANGE THIS
          4.0,
          1.0,
          Constants.ELEVATOR.GEARING,
          Constants.ELEVATOR.GEARING,
          ELEVATOR.MIN_HEIGHT,
          ELEVATOR.MAX_HEIGHT,
          Inches.of(0.5));
  }

  public Command setHeightCommand(Distance height) {
    return this.run(() -> setTargetHeightAndRun(height)).until(this::doneMoving);
  }

  public Command up() {
    return setHeightCommand(getHeight().plus(Inches.of(1)));
  }

  public Command down() {
    return setHeightCommand(getHeight().minus(Inches.of(1)));
  }

  public Command L1() {
    return setHeightCommand(ELEVATOR.L1_HEIGHT);
  }

  public Command L2() {
    return setHeightCommand(ELEVATOR.L2_HEIGHT);
  }

  public Command L3() {
    return setHeightCommand(ELEVATOR.L3_HEIGHT);
  }

  public Command L4() {
    return setHeightCommand(ELEVATOR.L4_HEIGHT);
  }

  public Command stow() {
    return setHeightCommand(ELEVATOR.STOW_HEIGHT);
  }

  public void setTargetHeightAndRun(Distance height) {
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
