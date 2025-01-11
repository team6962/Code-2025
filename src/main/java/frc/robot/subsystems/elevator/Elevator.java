package frc.robot.subsystems.elevator;


import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ELEVATOR;;

public class Elevator extends SubsystemBase {
    public static enum State {
        UP,
        DOWN,
        SLOW_UP,
        SLOW_DOWN,
        OFF
    }

    private TalonFX motor1;
    private DutyCycleEncoder encoder;
    private TalonFX motor2;
    private State state = State.OFF;
    private double currentElevatorHeight;

  public Elevator() {
    motor1 = new TalonFX(CAN.INTAKE);
    encoder = new DutyCycleEncoder(DIO.ELEVATOR_ENCODER);
    motor2 = new TalonFX(CAN.INTAKE);
    
    // encoder.setDistancePerRotation(1.0); // Set distance per rotation to 1.0
    // Configure the TalonFX using Phoenix 6 configuration
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Voltage.PeakForwardVoltage = 12.0; // Example voltage limit
    motorConfig.Voltage.PeakReverseVoltage = -12.0;

    // motor1.getConfigurator().apply((new FeedbackConfigs()).withFusedCANcoder(encoder));

    motor1.getConfigurator().apply(motorConfig);
    motor2.getConfigurator().apply(motorConfig);
    currentElevatorHeight = 0;
  }

  public Command setState(State state) {
    return runEnd(
      () -> this.state = state,
      () -> this.state = State.OFF
    );
  }

  @Override
  public void periodic() {

    currentElevatorHeight = encoder.get();

    switch (state) {
        case UP:
            if(currentElevatorHeight > ELEVATOR.ELEVATOR_MAX_HEIGHT) break;
            motor1.setControl(new DutyCycleOut(Preferences.ELEVATOR.POWER));
            motor2.setControl(new DutyCycleOut(Preferences.ELEVATOR.POWER));
            break;
        case DOWN:
            if(currentElevatorHeight < ELEVATOR.ELEVATOR_MIN_HEIGHT) break;
            motor1.setControl(new DutyCycleOut(-Preferences.ELEVATOR.POWER));
            motor2.setControl(new DutyCycleOut(-Preferences.ELEVATOR.POWER));
            break;
        case SLOW_UP:
            if(currentElevatorHeight > ELEVATOR.ELEVATOR_MAX_HEIGHT) break;
            motor1.setControl(new DutyCycleOut(Preferences.ELEVATOR.SLOW_POWER));
            motor2.setControl(new DutyCycleOut(Preferences.ELEVATOR.SLOW_POWER));
            break;
        case SLOW_DOWN:
            if(currentElevatorHeight < ELEVATOR.ELEVATOR_MIN_HEIGHT) break;
            motor1.setControl(new DutyCycleOut(-Preferences.ELEVATOR.SLOW_POWER));
            motor2.setControl(new DutyCycleOut(-Preferences.ELEVATOR.SLOW_POWER));
            break;
        case OFF:
            motor1.stopMotor();
            motor2.stopMotor();
            break;
        }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

