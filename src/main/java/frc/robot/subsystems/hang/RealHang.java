package frc.robot.subsystems.hang;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.HANG;

public class RealHang extends SubsystemBase implements Hang {
  protected TalonFX motor;
  protected DutyCycleEncoder encoder;
  protected PIDController pidController;

  public RealHang() {
    motor = new TalonFX(CAN.HANG);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(HANG.MAX_CURRENT));

    motor.getConfigurator().apply(config);

    encoder = new DutyCycleEncoder(DIO.HANG_ENCODER, 1.0, HANG.ENCODER_OFFSET.in(Rotations));

    pidController =
        new PIDController(HANG.DEPLOY_PROFILE.kP, HANG.DEPLOY_PROFILE.kI, HANG.DEPLOY_PROFILE.kD);
    pidController.setTolerance(Degrees.of(2).in(Rotations));
  }

  public Angle getAngle() {
    return Rotations.of(encoder.get());
  }

  private void setSpeed(double speed) {
    motor.set(speed);
  }

  private Command hangFast(Angle targetAngle) {
    return Commands.run(() -> setSpeed(1.0)).until(() -> getAngle().gte(targetAngle));
  }

  private Command hangSlow(Angle targetAngle) {
    return Commands.run(() -> setSpeed(0.5))
        .until(() -> getAngle().gte(HANG.SWAP_ANGLE) || getAngle().gte(targetAngle));
  }

  private Command deployFast(Angle targetAngle) {
    return Commands.run(() -> setSpeed(-1.0))
        .until(() -> getAngle().lte(HANG.SWAP_ANGLE) || getAngle().lte(targetAngle));
  }

  private Command deployFine(Angle targetAngle) {
    return Commands.run(
            () -> {
              double speed =
                  pidController.calculate(getAngle().in(Rotations), targetAngle.in(Rotations));

              setSpeed(speed);
            })
        .until(() -> pidController.atSetpoint());
  }

  private Command hang(Angle targetAngle) {
    return hangSlow(targetAngle)
        .onlyIf(() -> getAngle().lte(HANG.SWAP_ANGLE))
        .andThen(hangFast(targetAngle));
  }

  private Command deploy(Angle targetAngle) {
    return deployFast(targetAngle)
        .onlyIf(() -> getAngle().gte(HANG.SWAP_ANGLE))
        .andThen(deployFine(targetAngle));
  }

  private Command move(Angle targetAngle) {
    return defer(() -> getAngle().gte(targetAngle) ? deploy(targetAngle) : hang(targetAngle));
  }

  @Override
  public Command hang() {
    return move(HANG.HANG_ANGLE);
  }

  @Override
  public Command deploy() {
    return move(HANG.DEPLOY_ANGLE);
  }

  @Override
  public Command stow() {
    return move(HANG.STOW_ANGLE);
  }
}
