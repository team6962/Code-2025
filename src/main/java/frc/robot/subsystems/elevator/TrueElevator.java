package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ELEVATOR;
import frc.robot.util.hardware.motion.BasedElevator;

public class TrueElevator extends BasedElevator implements Elevator{
    public TrueElevator() {
        super(
            new Config(
                ELEVATOR.PROFILE.kP,
                ELEVATOR.PROFILE.kI,
                ELEVATOR.PROFILE.kD,
                ELEVATOR.PROFILE.kS,
                ELEVATOR.PROFILE.kG,
                ELEVATOR.PROFILE.kV,
                ELEVATOR.PROFILE.kA,
                ELEVATOR.MAX_UPWARD_VELOCITY,
                ELEVATOR.MAX_UPWARD_ACCELERATION,
                ELEVATOR.MAX_DOWNWARD_VELOCITY,
                ELEVATOR.MAX_DOWNWARD_ACCELERATION,
                ELEVATOR.MIN_HEIGHT,
                ELEVATOR.MAX_HEIGHT,
                ELEVATOR.GEARING, // gearReduction
                /*CHANGE*/ELEVATOR.CYCLE_HEIGHT, // sprocketRadius
                new MotorConfig[] {
                    new MotorConfig("LEFT_ELEVATOR_MOTOR", CAN.ELEVATOR_LEFT, ELEVATOR.LEFT_INVERTED), // Left Motor
                    new MotorConfig("RIGHT_ELEVATOR_MOTOR", CAN.ELEVATOR_RIGHT, ELEVATOR.RIGHT_INVERTED) // Right Motor
                },
                ELEVATOR.CURRENT.FREE_LIMIT, // freeCurrentLimit
                ELEVATOR.CURRENT.STALL_LIMIT, // stallCurrentLimit
                ELEVATOR.TOLERANCE, // tolerance
                new LimitSwitchConfig(DIO.ELEVATOR_FLOOR_LIMIT, LimitSwitchWiring.NormallyOpen, true), // floorLimitSwitch
                new LimitSwitchConfig(DIO.ELEVATOR_CEIL_LIMIT, LimitSwitchWiring.NormallyClosed, false) // ceilLimitSwitch
            )
        );
    }

    @Override
    public Command stow() {
      return moveTo(ELEVATOR.STOW_HEIGHT);
    }
  
    @Override
    public Command coralL1() {
      return moveTo(ELEVATOR.CORAL.L1_HEIGHT);
    }
  
    @Override
    public Command coralL2() {
      return moveTo(ELEVATOR.CORAL.L2_HEIGHT);
    }
  
    @Override
    public Command coralL3() {
      return moveTo(ELEVATOR.CORAL.L3_HEIGHT);
    }
  
    @Override
    public Command coralL4() {
      return moveTo(ELEVATOR.CORAL.L4_HEIGHT);
    }
  
    @Override
    public Command coralIntake() {
      return moveTo(ELEVATOR.CORAL.INTAKE_HEIGHT);
    }
  
    @Override
    public Command algaeGround() {
      return moveTo(ELEVATOR.ALGAE.GROUND_HEIGHT);
    }
  
    @Override
    public Command algaeL2() {
      return moveTo(ELEVATOR.ALGAE.L2_HEIGHT);
    }
  
    @Override
    public Command algaeL3() {
      return moveTo(ELEVATOR.ALGAE.L3_HEIGHT);
    }
  
    @Override
    public Command algaeBarge() {
      return moveTo(ELEVATOR.ALGAE.BARGE_HEIGHT);
    }
  
    @Override
    public Command algaeProcessor() {
      return moveTo(ELEVATOR.ALGAE.PROCESSOR_HEIGHT);
    }

    @Override
    public Distance getAverageHeight() {
        return this.getPosition();
    }

    @Override
    public Distance getMaxHeight() {
        return this.getMaxPosition();
    }

    @Override
    public Distance getMinHeight() {
        return this.getMinPosition();
    }

    @Override
    public Command move(double dutyCycle) {
        return this.applyDutyCycleWithGravityCompensation(dutyCycle);
    }

    @Override
    public Command up() {
        return this.applyDutyCycleWithGravityCompensation(ELEVATOR.FINE_CONTROL_DUTY_CYCLE);
    }

    @Override
    public Command down() {
        return this.applyDutyCycleWithGravityCompensation(-ELEVATOR.FINE_CONTROL_DUTY_CYCLE);
    }
}
