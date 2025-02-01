package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ELEVATOR;
import frc.robot.util.hardware.MotionControl.LinearController;

public class Elevator extends SubsystemBase {
  public static enum State {
    UP,
    DOWN,
    SLOW_UP,
    SLOW_DOWN,
    OFF
  }

  private LinearController controller;
  private SparkMax motorLeft = new SparkMax(CAN.ELEVATOR_LEFT, MotorType.kBrushless);
  private SparkMax motorRight = new SparkMax(CAN.ELEVATOR_RIGHT, MotorType.kBrushless);
  private State state = State.OFF;
  private Distance currentHeight = null;
  private Distance targetHeight = null;

  public Elevator() {
    controller =
        new LinearController(
            this,
            motorLeft,
            DIO.ELEVATOR_ENCODER,
            0124124, // CHANGE
            4.0,
            4.0,
            ELEVATOR.GEARING,
            ELEVATOR.ELEVATOR_MIN_HEIGHT,
            ELEVATOR.ELEVATOR_MAX_HEIGHT,
            Inches.of(0.5),
            false);

    SparkMaxConfig config = new SparkMaxConfig();
    config.follow(motorLeft, true);

    motorRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command setState(State state) {
    return runEnd(() -> this.state = state, () -> this.state = State.OFF);
  }

  @Override
  public void periodic() {
    controller.run();
  }

  public void setElevatorHeight(Distance height) {
    targetHeight = height;
    controller.setTargetHeight(height);
  }

  public Distance getCurrentHeight() {
    return currentHeight;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
