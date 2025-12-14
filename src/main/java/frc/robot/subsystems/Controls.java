package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.XBoxSwerve;
import frc.robot.constants.Constants.DEVICES;

public class Controls {
  public final CommandXboxController operator =
      new CommandXboxController(DEVICES.OPERATOR_XBOX_CONTROLLER);
  public final CommandXboxController driver =
      new CommandXboxController(DEVICES.DRIVE_XBOX_CONTROLLER);
  private XBoxSwerve xBoxSwerve;

  public Controls(SwerveDrive swerveDrive) {
    XboxController driverHID = driver.getHID();
    xBoxSwerve = new XBoxSwerve(swerveDrive, driverHID);

    swerveDrive.setDefaultCommand(xBoxSwerve);
  }

  public XBoxSwerve getSwerveController() {
    return xBoxSwerve;
  }

  public void configureBindings(SwerveDrive swerveDrive) {
    driver.a();
    driver.b();
    driver.x();
    driver.y();
    driver.start();
    driver.back();
    driver.leftBumper();
    driver.rightBumper();
    driver.rightStick();
    driver.leftStick();
    driver.povCenter(); // USED
    driver.povUp(); // USED
    driver.povDown(); // USED
    driver.povLeft(); // USED
    driver.povRight(); // USED
    driver.leftTrigger(); // USED
    driver.rightTrigger(); // USED

    XboxController driverHID = driver.getHID();
    XboxController operatorHID = operator.getHID();

    Logger.logXBoxController("Controllers/Driver", driverHID);
    Logger.logXBoxController("Controllers/Operator", operatorHID);

    operator.a();
    operator.b();
    operator.x();
    operator.y();

    operator.povUp();
    operator.povDown();
    operator.povRight();
    operator.povLeft();
    operator.back();
    operator.start();
    operator.leftStick();
    operator.rightStick();
    operator.rightBumper();
    operator.rightTrigger();
    operator.leftBumper();
    operator.leftTrigger();
  }

  private Command rumble(CommandXboxController controller) {
    return Commands.runEnd(
            () -> {
              controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            },
            () -> {
              controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            })
        .withTimeout(0.25);
  }

  private Command rumble(CommandXboxController controller, BooleanSupplier booleanSupplier) {
    return Commands.runEnd(
        () -> {
          if (booleanSupplier.getAsBoolean()) {
            controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          } else {
            controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          }
        },
        () -> {
          controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  public Command rumbleDriver() {
    return rumble(driver);
  }

  public Command rumbleDriver(BooleanSupplier booleanSupplier) {
    return rumble(driver, booleanSupplier);
  }

  public Command rumbleOperator() {
    return rumble(operator);
  }

  public Command rumbleOperator(BooleanSupplier booleanSupplier) {
    return rumble(operator, booleanSupplier);
  }

  public Command rumbleBoth() {
    return rumbleOperator().alongWith(rumbleDriver());
  }

  public Command rumbleBoth(BooleanSupplier booleanSupplier) {
    return rumbleOperator(booleanSupplier).alongWith(rumbleDriver(booleanSupplier));
  }
}
