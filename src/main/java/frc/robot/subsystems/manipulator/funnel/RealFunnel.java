package frc.robot.subsystems.manipulator.funnel;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.StatusChecks;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.CAN;
import frc.robot.constants.Constants.MANIPULATOR;
import frc.robot.util.hardware.SparkMaxUtil;

public class RealFunnel extends Funnel {
  private final SparkMax motor;

  public RealFunnel() {
    motor = new SparkMax(CAN.MANIPULATOR_FUNNEL, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();

    SparkMaxUtil.configure(config, true, IdleMode.kBrake);
    SparkMaxUtil.saveAndLog(this, motor, config);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(stop());

    StatusChecks.under(this).add("motor", motor);
  }

  public Command runSpeed(double speed) {
    return this.run(() -> motor.set(speed));
  }

  @Override
  public Command intake() {
    return runSpeed(MANIPULATOR.FUNNEL_IN_SPEED);
  }

  @Override
  public Command forwards() {
    return runSpeed(MANIPULATOR.FUNNEL_IN_SPEED);
  }

  @Override
  public Command stop() {
    return runSpeed(0);
  }
}
