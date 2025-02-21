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
    public void test() {
        Distance length = CoralSequences.estimatePathTranslationLength(
            ReefPositioning.REEF_CENTER.minus(new Translation2d(-2, 0)),
            ReefPositioning.REEF_CENTER.minus(new Translation2d(2, 0))
        );

        Distance perfectLength = Meters.of(4);
        Distance reefRadius = CoralSequences.ESTIMATED_REEF_AVOIDANCE_DIAMETER.div(2);
        Distance expectedLength = perfectLength.minus(reefRadius.times(2)).plus(reefRadius.times(Math.PI));

        assertEquals(perfectLength.in(Meters), length.in(Meters), 0.01);

        assertEquals(expectedLength.in(Meters), length.in(Meters), 0.01);
    }
}
