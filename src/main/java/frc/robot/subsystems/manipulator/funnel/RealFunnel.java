package frc.robot.subsystems.manipulator.funnel;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.StatusChecks;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.MANIPULATOR;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.subsystems.manipulator.grabber.Grabber;

public class RealFunnel extends Funnel{
    private final SparkMax motor;
    
    public RealFunnel(){
        motor = new SparkMax(CAN.MANIPULATOR_FUNNEL, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();

        SparkMaxUtil.configure(config, false, IdleMode.kBrake);
        SparkMaxUtil.saveAndLog(this, motor, config);
        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setDefaultCommand(stop());

        StatusChecks.under(this).add("motor", motor);
    }

    public Command runSpeed(double speed) {
        return this.run(() -> motor.set(speed));
    }

    public Command intake(Grabber grabber) {
        return runSpeed(MANIPULATOR.FUNNEL_IN_SPEED).until(grabber::hasCoral);
    }

    public Command forwards() {
        return runSpeed(MANIPULATOR.FUNNEL_IN_SPEED);
    }

    public Command stop() {
        return runSpeed(0);
    }
}
