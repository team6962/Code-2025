package com.team6962.lib.swerve.module;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;

import frc.robot.commands.auto.AutoPaths;
import frc.robot.commands.auto.PathOutlineFactory;

public class PathFactoryCountTest {
    private void countTest(int expectedCount, AutoPaths.PlanConstraints constraints) {
        PathOutlineFactory factory = new PathOutlineFactory(constraints);

        int count = factory.getPathCount();

        assertEquals(expectedCount, count);
    }

    @Test
    public void countTest1() {
        countTest(1 * 2 * 2 * 1, new AutoPaths.PlanConstraints(
            List.of(
                new AutoPaths.CoralPosition(4, 2),
                new AutoPaths.CoralPosition(6, 3)
            ),
            new AutoPaths.CoralSources(true, true, true)
        ));
    }

    @Test
    public void countTest2() {
        countTest(2 * 2 * 2 * 1, new AutoPaths.PlanConstraints(
            List.of(
                new AutoPaths.CoralPosition(4, 2),
                new AutoPaths.CoralPosition(6, 3)
            ),
            new AutoPaths.CoralSources(false, true, true)
        ));
    }

    @Test
    public void countTestNoPositions() {
        countTest(1, new AutoPaths.PlanConstraints(
            List.of(),
            new AutoPaths.CoralSources(false, true, true)
        ));
    }

    @Test
    public void countTestNoStationsNoPreplaced() {
        countTest(1, new AutoPaths.PlanConstraints(
            List.of(
                new AutoPaths.CoralPosition(4, 2),
                new AutoPaths.CoralPosition(6, 3)
            ),
            new AutoPaths.CoralSources(false, false, false)
        ));
    }

    @Test
    public void countTestNoStationsWithPreplaced() {
        countTest(2, new AutoPaths.PlanConstraints(
            List.of(
                new AutoPaths.CoralPosition(4, 2),
                new AutoPaths.CoralPosition(6, 3)
            ),
            new AutoPaths.CoralSources(true, false, false)
        ));
    }
}
