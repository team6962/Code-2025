package frc.robot.commands.safeties;

import static edu.wpi.first.units.Units.Rotations;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.pivot.ManipulatorPivot;
import frc.robot.subsystems.manipulator.pivot.RealManipulatorPivot;

public class OptimizedSafeties {
    private Elevator elevator;
    private ManipulatorPivot pivot;

    private static final List<Pair<MechanismRange<DistanceUnit>, MechanismRange<AngleUnit>>> SAFE_RANGES;
    private static final MechanismRange<AngleUnit> MAX_RANGE;

    static {
        SAFE_RANGES = new ArrayList<>();
        MAX_RANGE = MechanismRange.union(SAFE_RANGES.stream().map(pair -> pair.getSecond()).toList());
    }

    public OptimizedSafeties(Elevator elevator, ManipulatorPivot pivot) {
        this.elevator = elevator;
        this.pivot = pivot;
    }

    private MechanismRange<AngleUnit> getSafeRange(MechanismRange<DistanceUnit> elevatorHeight) {
        MechanismRange<AngleUnit> safeRange = MAX_RANGE;

        for (Pair<MechanismRange<DistanceUnit>, MechanismRange<AngleUnit>> pair : SAFE_RANGES) {
            if (pair.getFirst().intersectsWith(elevatorHeight)) {
                safeRange = safeRange.intersection(pair.getSecond());
            }
        }

        return safeRange;
    }

    private MechanismRange<AngleUnit> getSafeRange(Distance elevatorHeight) {
        for (Pair<MechanismRange<DistanceUnit>, MechanismRange<AngleUnit>> pair : SAFE_RANGES) {
            if (pair.getFirst().contains(elevatorHeight)) {
                return pair.getSecond();
            }
        }

        return MAX_RANGE;
    }

    private Command moveElevatorPrecise(Distance position) {
        return elevator.setHeight(position);
    }

    private Command movePivotPrecise(Angle targetPosition) {
        return pivot.pivotTo(() -> targetPosition);
    }

    private Command movePivotOvershoot(Angle targetPosition) {
        if (!(pivot instanceof RealManipulatorPivot)) {
            return movePivotPrecise(targetPosition);
        }

        return Commands.defer(() -> {
            double sign = Math.signum(targetPosition.minus(pivot.getAngle()).abs(Rotations));

            return pivot.run(() -> pivot.pivotSpeed(sign * 1.0))
                .until(() -> Math.signum(pivot.getAngle().minus(targetPosition).in(Rotations)) == sign);
        }, Set.of(pivot));
    }

    public Command move(Distance elevatorTarget, Angle pivotTarget) {
        return Commands.defer(() -> {
            MechanismRange<DistanceUnit> elevatorRange = new MechanismRange<>(elevator.getAverageHeight(), elevatorTarget);
            MechanismRange<AngleUnit> pivotRange = getSafeRange(elevatorRange);
            MechanismRange<AngleUnit> endRange = getSafeRange(elevatorTarget);

            return Commands.sequence(
                movePivotOvershoot(MeasureMath.castAngle(pivotRange.clamp(pivot.getAngle()))),
                Commands.parallel(
                    moveElevatorPrecise(elevatorTarget),
                    movePivotPrecise(MeasureMath.castAngle(pivotRange.clamp(pivotTarget)))
                ),
                movePivotPrecise(MeasureMath.castAngle(endRange.clamp(pivot.getAngle())))
            );
        }, Set.of(elevator, pivot));
    }
}
