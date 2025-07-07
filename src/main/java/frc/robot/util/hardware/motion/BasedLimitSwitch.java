package frc.robot.util.hardware.motion;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.wpilibj.DigitalInput;

public class BasedLimitSwitch {
    private BasedElevator.LimitSwitchConfig config;
    private DigitalInput input;

    public BasedLimitSwitch(BasedElevator.LimitSwitchConfig config) {
        this.config = config;
        this.input = new DigitalInput(config.dio);
    }

    public boolean isPressed() {
        if (config.wiring == BasedElevator.LimitSwitchWiring.NormallyOpen) {
            return !input.get();
        } else {
            return input.get();
        }
    }

    public void logUnder(String path) {
        Logger.logBoolean(path, this::isPressed);
        Logger.logBoolean(path + "Raw", input::get);
    }
}
