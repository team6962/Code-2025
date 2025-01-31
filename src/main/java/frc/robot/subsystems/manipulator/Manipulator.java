package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences.MANIPULATOR;

public class Manipulator extends SubsystemBase {
    public final ManipulatorPivot pivot;
    public final ManipulatorGrabber algae;
    public final ManipulatorGrabber coral;

    public Manipulator() {
        pivot = new ManipulatorPivot();

        algae = new ManipulatorGrabber(
            Constants.CAN.MANIPULATOR_ALGAE,
            new ManipulatorGrabber.DigitalSensor(Constants.DIO.ALGAE_BEAM_BREAK),
            MANIPULATOR.ALGAE_IN_SPEED,
            MANIPULATOR.ALGAE_OUT_SPEED,
            () -> ENABLED_SYSTEMS.ENABLE_MANIPULATOR
        );

        coral = new ManipulatorGrabber(
            Constants.CAN.MANIPULATOR_CORAL,
            new ManipulatorGrabber.TimeSensor(false, Seconds.of(1), Seconds.of(1)),
            MANIPULATOR.CORAL_IN_SPEED,
            MANIPULATOR.CORAL_OUT_SPEED,
            () -> ENABLED_SYSTEMS.ENABLE_MANIPULATOR
        );
    }
}
