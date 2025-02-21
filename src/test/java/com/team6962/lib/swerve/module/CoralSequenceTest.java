package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ReefPositioning;
import frc.robot.commands.autonomous.CoralSequences;

public class CoralSequenceTest {
    @Test
    public void simpleIntersectTest() {
        Distance length = CoralSequences.estimatePathTranslationLength(
            ReefPositioning.REEF_CENTER.plus(new Translation2d(-3, 0)),
            ReefPositioning.REEF_CENTER.plus(new Translation2d(3, 0))
        );

        Distance perfectLength = Meters.of(6);
        Distance reefRadius = CoralSequences.ESTIMATED_REEF_AVOIDANCE_DIAMETER.div(2);
        Distance expectedLength = perfectLength.minus(CoralSequences.ESTIMATED_REEF_AVOIDANCE_DIAMETER).plus(reefRadius.times(Math.PI));

        assertEquals(expectedLength.in(Meters), length.in(Meters), 0.01);
    }

    @Test
    public void linedUpTest() {
        Distance length = CoralSequences.estimatePathTranslationLength(
            ReefPositioning.REEF_CENTER.plus(new Translation2d(-5, 0)),
            ReefPositioning.REEF_CENTER.plus(new Translation2d(-2, 0))
        );

        assertEquals(3, length.in(Meters), 0.01);
    }

    @Test
    public void diagonalIntersectionTest() {
        Distance reefRadius = CoralSequences.ESTIMATED_REEF_AVOIDANCE_DIAMETER.div(2);

        Translation2d start = new Translation2d(-3, -reefRadius.in(Meters) / 2.0);
        Translation2d end = new Translation2d(3, reefRadius.in(Meters) / 2.0);

        Distance length = CoralSequences.estimatePathTranslationLength(
            ReefPositioning.REEF_CENTER.plus(start),
            ReefPositioning.REEF_CENTER.plus(end)
        );

        Distance perfectLength = Meters.of(start.getDistance(end));
        Distance expectedLength = perfectLength.minus(CoralSequences.ESTIMATED_REEF_AVOIDANCE_DIAMETER).plus(reefRadius.times(Math.PI));

        assertEquals(expectedLength.in(Meters), length.in(Meters), 0.01);
    }

    @Test
    public void noIntersectionTest() {
        Distance length = CoralSequences.estimatePathTranslationLength(
            ReefPositioning.REEF_CENTER.plus(new Translation2d(-5, 3)),
            ReefPositioning.REEF_CENTER.plus(new Translation2d(5, 3))
        );

        assertEquals(10, length.in(Meters), 0.01);
    }
}
