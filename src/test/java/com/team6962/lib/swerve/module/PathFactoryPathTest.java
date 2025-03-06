package com.team6962.lib.swerve.module;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;

import frc.robot.commands.auto.AutoPaths;
import frc.robot.commands.auto.PathOutlineFactory;

public class PathFactoryPathTest {
    private void pathTest(List<AutoPaths.CoralMovement> expected, int index, AutoPaths.PlanConstraints constraints) {
        PathOutlineFactory factory = new PathOutlineFactory(constraints);

        List<AutoPaths.CoralMovement> path = factory.getPath(index);

        assertEquals(expected, path);
    }

    @Test
    public void pathTest1() {
        pathTest(
            List.of(
                new AutoPaths.CoralMovement(new AutoPaths.CoralPosition(4, 2), AutoPaths.CoralSource.PREPLACED),
                new AutoPaths.CoralMovement(new AutoPaths.CoralPosition(6, 3), AutoPaths.CoralSource.STATION_LEFT)
            ),
            0,
            new AutoPaths.PlanConstraints(
                List.of(
                    new AutoPaths.CoralPosition(4, 2),
                    new AutoPaths.CoralPosition(6, 3)
                ),
                new AutoPaths.CoralSources(true, true, true)
            )
        );
    }

    @Test
    public void pathTest2() {
        pathTest(
            List.of(
                new AutoPaths.CoralMovement(new AutoPaths.CoralPosition(6, 3), AutoPaths.CoralSource.PREPLACED),
                new AutoPaths.CoralMovement(new AutoPaths.CoralPosition(4, 2), AutoPaths.CoralSource.STATION_RIGHT)
            ),
            1,
            new AutoPaths.PlanConstraints(
                List.of(
                    new AutoPaths.CoralPosition(4, 2),
                    new AutoPaths.CoralPosition(6, 3)
                ),
                new AutoPaths.CoralSources(true, false, true)
            )
        );
    }

    @Test
    public void pathTest3() {
        pathTest(
            List.of(),
            0,
            new AutoPaths.PlanConstraints(
                List.of(),
                new AutoPaths.CoralSources(true, true, true)
            )
        );
    }

    @Test
    public void pathTest4() {
        pathTest(
            List.of(),
            0,
            new AutoPaths.PlanConstraints(
                List.of(new AutoPaths.CoralPosition(6, 3)),
                new AutoPaths.CoralSources(false, false, false)
            )
        );
    }
}
