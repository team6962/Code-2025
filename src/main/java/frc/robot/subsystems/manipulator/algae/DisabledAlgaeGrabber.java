package frc.robot.subsystems.manipulator.algae;

import static edu.wpi.first.units.Units.Seconds;

public class DisabledAlgaeGrabber extends SimAlgaeGrabber {
    public DisabledAlgaeGrabber() {
        super(Seconds.of(0), Seconds.of(0), Seconds.of(0), Seconds.of(0));
    }
}
