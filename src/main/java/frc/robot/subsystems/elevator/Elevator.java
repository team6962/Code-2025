package frc.robot.subsystems.elevator;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Constants.CAN;

public class Elevator extends SubsystemBase {
    public static enum State {
        UP,
        DOWN,
        SLOW_UP,
        SLOW_DOWN,
        OFF
    }

    private TalonFX motor;
    private State state = State.OFF;
    private DutyCycleEncoder encoder;


  public Elevator() {
    motor = new TalonFX(CAN.INTAKE);
    encoder = new DutyCycleEncoder(CAN.ELEVATOR_ENCODER);
    // encoder.setDistancePerRotation(1.0); // Set distance per rotation to 1.0
    // Configure the TalonFX using Phoenix 6 configuration
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Voltage.PeakForwardVoltage = 12.0; // Example voltage limit
    motorConfig.Voltage.PeakReverseVoltage = -12.0;
    motor.getConfigurator().apply(motorConfig);

  }

  public Command setState(State state) {
    return runEnd(
      () -> this.state = state,
      () -> this.state = State.OFF
    );
  }

  @Override
  public void periodic() {
    switch (state) {
        case UP:
            motor.setControl(new DutyCycleOut(Preferences.ELEVATOR.POWER));
            break;
        case DOWN:
            motor.setControl(new DutyCycleOut(-Preferences.ELEVATOR.POWER));
            break;
        case SLOW_UP:
            motor.setControl(new DutyCycleOut(Preferences.ELEVATOR.SLOW_POWER));
            break;
        case SLOW_DOWN:
            motor.setControl(new DutyCycleOut(-Preferences.ELEVATOR.SLOW_POWER));
            break;
        case OFF:
            motor.stopMotor();
            break;
        }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

