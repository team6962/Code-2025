package frc.robot.util.hardware.motion;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BasedMotor extends SubsystemBase {
    public BasedElevator.Config motorConfig;
    public String name;

    public BasedMotor(BasedElevator.Config elevatorConfig, BasedElevator.MotorConfig motorConfig){
        this.motorConfig = motorConfig;
    }   
}
