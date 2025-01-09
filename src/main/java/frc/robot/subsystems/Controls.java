package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.utils.FactoryCommand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Constants.DEVICES;
import frc.robot.commands.drive.XBoxSwerve;

public class Controls {
  public static final CommandXboxController operator = new CommandXboxController(DEVICES.OPERATOR_XBOX_CONTROLLER);
  public static final CommandXboxController driver = new CommandXboxController(DEVICES.DRIVE_XBOX_CONTROLLER);

  public static void configureBindings(
      RobotStateController stateController,
      SwerveDrive swerveDrive
)
    {

    driver.a();
    driver.b();
    driver.x();
    driver.y(); // USED
    driver.start();
    driver.back();
    driver.leftBumper();
    driver.rightBumper();
    driver.leftStick();
    driver.rightStick();
    driver.povCenter(); // USED
    driver.povUp(); // USED
    driver.povDown(); // USED
    driver.povLeft(); // USED
    driver.povRight(); // USED
    driver.leftTrigger(); // USED
    driver.rightTrigger(); // USED
    swerveDrive.setDefaultCommand(new XBoxSwerve(swerveDrive, driver.getHID(), stateController));    

    if (RobotBase.isSimulation()) {
      driver.button(1).whileTrue(new FactoryCommand(() ->
        swerveDrive.pathfindTo(frc.robot.Constants.Field.AUTO_MOVE_POSITIONS.get("AMP").get())
      ));
      
      // driver.button(1).whileTrue(stateController.setState(RobotStateController.State.AIM_SPEAKER).alongWith(stateController.setState(RobotStateController.State.SPIN_UP)));
    }

    operator.a();
    operator.b();
    operator.x();
    operator.y(); // USED
    operator.start();
    operator.back();
    operator.leftBumper();
    operator.rightBumper();
    operator.leftStick();
    operator.rightStick();
    operator.povCenter(); // USED
    operator.povUp(); // USED
    operator.povDown(); // USED
    operator.povLeft(); // USED
    operator.povRight(); // USED
    operator.leftTrigger(); // USED
    operator.rightTrigger(); // USED

    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Dashboard");

    // driverTab.addBoolean("Is Aimed", () -> shooter.isAimed())
    //   .withWidget(BuiltInWidgets.kBooleanBox)
    //   .withPosition(3, 0)
    //   .withSize(2, 2)
    //   .withProperties(Map.of("min", 0, "max", 100));

    // driverTab.addDouble("Battery Capacity", () -> Constants.SWERVE_DRIVE.BATTERY_VOLTAGE < RobotContainer.getVoltage() ? 100.0 : (RobotContainer.getTotalCurrent() / ((Constants.SWERVE_DRIVE.BATTERY_VOLTAGE - RobotContainer.getVoltage()) / (Constants.SWERVE_DRIVE.BATTERY_RESISTANCE)) * 100.0))
    //   .withWidget(BuiltInWidgets.kDial)
    //   .withPosition(3, 2)
    //   .withSize(2, 2)
    //   .withProperties(Map.of("min", 0, "max", 100));
  }

  private static Command rumble(CommandXboxController controller) {
    return Commands.runEnd(() -> {
      controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
      LEDs.setState(LEDs.State.GOOD);
    },
    () -> {
      controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }
    ).withTimeout(0.25);
  }

  private static Command rumble(CommandXboxController controller, BooleanSupplier booleanSupplier) {
    return Commands.runEnd(() -> {
      if (booleanSupplier.getAsBoolean()) {
        controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        LEDs.setState(LEDs.State.GOOD);
      } else {
        controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
      }
    },
    () -> {
      controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }
    );
  }

  public static Command rumbleDriver() {
    return rumble(driver);
  }

  public static Command rumbleDriver(BooleanSupplier booleanSupplier) {
    return rumble(driver, booleanSupplier);
  }

  public static Command rumbleOperator() {
    return rumble(operator);
  }

  public static Command rumbleOperator(BooleanSupplier booleanSupplier) {
    return rumble(operator, booleanSupplier);
  }

  public static Command rumbleBoth() {
    return rumbleOperator().alongWith(rumbleDriver());
  }

  public static Command rumbleBoth(BooleanSupplier booleanSupplier) {
    return rumbleOperator(booleanSupplier).alongWith(rumbleDriver(booleanSupplier));
  }
}
