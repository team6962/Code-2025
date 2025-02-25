package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.MANIPULATOR_PIVOT;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;

public class ManipulatorSafeties extends SubsystemBase {
    Manipulator manipulator;
    Elevator elevator;
    
    public ManipulatorSafeties(Elevator elevator, Manipulator manipulator) {
        this.elevator = elevator;
        this.manipulator = manipulator;
    }

    public static Angle calcSafeMinAngle(Distance elevatorHeight) {
       if (elevatorHeight.lt(Inches.of(43))) {
        return Degrees.of(-50.0);
       }

       if (Inches.of(43).lt(elevatorHeight) && elevatorHeight.lt(Inches.of(54))) {
        return Degrees.of(-90.0);
       }

       if (Inches.of(54).lt(elevatorHeight)) {
        return MANIPULATOR_PIVOT.MIN_ANGLE;
       }

       return MANIPULATOR_PIVOT.SAFE_ANGLE;
    }

    public static Angle calcSafeMaxAngle(Distance elevatorHeight) {
        if (elevatorHeight.lt(Inches.of(43))) {
            return MANIPULATOR_PIVOT.MAX_ANGLE;
        }

        if (Inches.of(43).lt(elevatorHeight) && elevatorHeight.lt(Inches.of(82))) {
            return Degrees.of(-20.0);
        }

        if (Inches.of(82).lt(elevatorHeight)) {
            return MANIPULATOR_PIVOT.ALGAE.BARGE_ANGLE;
        }

        return MANIPULATOR_PIVOT.SAFE_ANGLE;
    }
    
    @Override
    public void periodic() {
        Distance elevatorHeight = elevator.getAverageHeight();
        Angle minAngle = calcSafeMinAngle(elevatorHeight);
        Angle maxAngle = calcSafeMaxAngle(elevatorHeight);

        manipulator.pivot.setMinMaxAngle(
            minAngle,
            maxAngle);
    }
}