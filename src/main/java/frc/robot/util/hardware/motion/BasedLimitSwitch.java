package frc.robot.util.hardware.motion;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;

public class BasedLimitSwitch {
    private BasedElevator.LimitSwitchConfig config;
    private DigitalInput input;
    private Boolean simulatedState;

    public BasedLimitSwitch(BasedElevator.LimitSwitchConfig config) {
        this.config = config;
        this.input = new DigitalInput(config.dio);
    }

    public void setSimulatedState(Boolean simulatedState) {
        this.simulatedState = simulatedState;
    }

    public boolean isPressed() {
        if (simulatedState != null && RobotBase.isSimulation()) {
            return simulatedState;
        }

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
