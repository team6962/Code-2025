package frc.robot.subsystems.elevator;

import java.util.Set;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.hardware.motion.BasedElevator;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ELEVATOR;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.VOLTAGE_LADDER;

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

}
