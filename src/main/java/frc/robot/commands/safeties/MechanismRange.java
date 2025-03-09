package frc.robot.commands.safeties;

import java.util.List;

import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class MechanismRange<U extends Unit> {
    private final Measure<U> min;
    private final Measure<U> max;

    public MechanismRange(Measure<U> a, Measure<U> b) {
        if (a.lt(b)) {
            this.min = a;
            this.max = b;
        } else {
            this.min = b;
            this.max = a;
        }
    }

    public MechanismRange<U> union(MechanismRange<U> other) {
        return new MechanismRange<>(MeasureMath.min(this.min, other.min), MeasureMath.max(this.max, other.max));
    }

    public MechanismRange<U> intersection(MechanismRange<U> other) {
        return new MechanismRange<>(MeasureMath.max(this.min, other.min), MeasureMath.min(this.max, other.max));
    }

    public Measure<U> clamp(Measure<U> value) {
        return MeasureMath.clamp(value, min, max);
    }

    public boolean intersectsWith(MechanismRange<U> other) {
        return this.min.lt(other.max) && this.max.gt(other.min);
    }

    public boolean contains(Measure<U> value) {
        return this.min.lt(value) && this.max.gt(value);
    }

    public static <U extends Unit> MechanismRange<U> union(List<MechanismRange<U>> ranges) {
        MechanismRange<U> union = new MechanismRange<>(ranges.get(0).min, ranges.get(0).max);

        for (int i = 1; i < ranges.size(); i++) {
            union = union.union(ranges.get(i));
        }

        return union;
    }

    public static <U extends Unit> MechanismRange<U> intersection(List<MechanismRange<U>> ranges) {
        MechanismRange<U> intersection = new MechanismRange<>(ranges.get(0).min, ranges.get(0).max);

        for (int i = 1; i < ranges.size(); i++) {
            intersection = intersection.intersection(ranges.get(i));
        }

        return intersection;
    }
}
