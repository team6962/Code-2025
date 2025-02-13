package frc.robot.subsystems.hang;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import frc.robot.util.hardware.MotionControl.PivotController;

public class Hang extends PivotController {

  public Hang() {
    super(
        CAN.HANG,
        DIO.HANG_ENCODER,
        Constants.HANG_PIVOT.ENCODER_OFFSET.in(Rotations),
        Constants.HANG_PIVOT.PROFILE.kP,
        Constants.HANG_PIVOT.PROFILE.kS,
        Constants.HANG_PIVOT.GEARING,
        Preferences.HANG_PIVOT.MIN_ANGLE,
        Preferences.HANG_PIVOT.MAX_ANGLE,
        Degrees.of(2.0),
        false);
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.HANG) stopMotor();
    if (RobotContainer.getVoltage() < Preferences.VOLTAGE_LADDER.HANG) stopMotor();
  }

  public Command deploy() {
    return setTargetAngleCommand(Preferences.HANG_PIVOT.HANG_ANGLE);
  }

  public Command stow() {
    return setTargetAngleCommand(Preferences.HANG_PIVOT.STOW_ANGLE);
  }

  public Command stop() {
    return Commands.run(this::stopMotor);
  }

  public Command test() {
    return Commands.sequence(
      deploy(),
      stow()
    );
  }

  public Command setTargetAngleCommand(Angle angle) {
    if (!ENABLED_SYSTEMS.HANG) return stop();
    return this.run(() -> setAngle(angle)).until(this::doneMoving);
  }

  @Override
  public void simulationPeriodic() {}
}
