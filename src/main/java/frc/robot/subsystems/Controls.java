package frc.robot.subsystems;

import com.team6962.lib.swerve.SwerveDrive;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Constants.DEVICES;
import frc.robot.commands.autonomous.Autonomous;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class Controls {
  public static final CommandXboxController operator =
      new CommandXboxController(DEVICES.OPERATOR_XBOX_CONTROLLER);
  public static final CommandXboxController driver =
      new CommandXboxController(DEVICES.DRIVE_XBOX_CONTROLLER);

  public static void configureBindings(
      RobotStateController stateController,
      SwerveDrive swerveDrive,
      // Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Hang hang, Autonomous autonomous) {

    // Driver
    // Move swerve chassis
    // Rotate Swerve Chassis
    // ^^^ see xbox swerve.java

    // Button to move to processor
    // Button to move to source
    // Auto orient towards algae
    // Button to move to left/right reef (dpad left right)
    // Button for aligning to algae on the reef (dpad up)

    driver.a();
    driver.b();
    driver.x();
    driver.y();
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
      driver
          .button(1)
          .whileTrue(
              Commands.defer(
                  () ->
                    autonomous.reefPoleAlign(0),
                  Set.of()));
      driver
          .button(5)
          .onTrue(
              Commands.defer(
                  () ->
                      autonomous.reefPoleAlign(0),
                  Set.of()));

      // driver.button(1).whileTrue(stateController.setState(RobotStateController.State.AIM_SPEAKER).alongWith(stateController.setState(RobotStateController.State.SPIN_UP)));
    }

    // Operator
    // Button to L2-L4, and Barge Height
    // Intake Algae with The Box
    // Output Algae from The Box
    // Output Coral
    // Deploy Hang
    // Retract Hang
    // L2 Algae Removal height
    // L3 Algae Removal Height
    // Algae ground Height

    // operator.a().onTrue(elevator.coralL2());
    // operator.b().onTrue(elevator.coralL3());
    // operator.x().onTrue(elevator.coralL4());
    // operator.y().onTrue(elevator.algaeBarge());
    // operator.start().onTrue(elevator.stow()); // assume this is processor height
    // operator.back().onTrue(elevator.algaeGround());
    // operator.leftStick().onTrue(elevator.algaeL2());
    // operator.rightStick().onTrue(elevator.algaeL3());
    // operator.povCenter();
    // operator.povUp().whileTrue(elevator.up());
    // operator.povDown().whileTrue(elevator.down());
    operator.povLeft().whileTrue(hang.deploy());
    operator.povRight().whileTrue(hang.stow());
    operator.leftBumper().onTrue(manipulator.coral.intake());
    operator.rightBumper().onTrue(manipulator.coral.drop());
    operator.leftTrigger().onTrue(manipulator.algae.intake());
    operator.rightTrigger().onTrue(manipulator.algae.drop());
    operator.y().onTrue(autonomous.reefPoleAlign(0));

    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Dashboard");

    // driverTab.addBoolean("Is Aimed", () -> shooter.isAimed())
    //   .withWidget(BuiltInWidgets.kBooleanBox)
    //   .withPosition(3, 0)
    //   .withSize(2, 2)
    //   .withProperties(Map.of("min", 0, "max", 100));

    // driverTab.addDouble("Battery Capacity", () -> Constants.SWERVE_DRIVE.BATTERY_VOLTAGE <
    // RobotContainer.getVoltage() ? 100.0 : (RobotContainer.getTotalCurrent() /
    // ((Constants.SWERVE_DRIVE.BATTERY_VOLTAGE - RobotContainer.getVoltage()) /
    // (Constants.SWERVE_DRIVE.BATTERY_RESISTANCE)) * 100.0))
    //   .withWidget(BuiltInWidgets.kDial)
    //   .withPosition(3, 2)
    //   .withSize(2, 2)
    //   .withProperties(Map.of("min", 0, "max", 100));
  }

  private static Command rumble(CommandXboxController controller) {
    return Commands.runEnd(
            () -> {
              controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
              LEDs.setState(LEDs.State.GOOD);
            },
            () -> {
              controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            })
        .withTimeout(0.25);
  }

  private static Command rumble(CommandXboxController controller, BooleanSupplier booleanSupplier) {
    return Commands.runEnd(
        () -> {
          if (booleanSupplier.getAsBoolean()) {
            controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            LEDs.setState(LEDs.State.GOOD);
          } else {
            controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          }
        },
        () -> {
          controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
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
