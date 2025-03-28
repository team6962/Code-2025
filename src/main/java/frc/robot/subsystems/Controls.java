package frc.robot.subsystems;

import com.team6962.lib.swerve.SwerveDrive;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Constants.DEVICES;
import frc.robot.auto.utils.AutonomousCommands;
import frc.robot.commands.PieceCombos;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.LEDs.LEDs.State;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.manipulator.Manipulator;
import java.util.function.BooleanSupplier;

public class Controls {
  public static final CommandXboxController operator =
      new CommandXboxController(DEVICES.OPERATOR_XBOX_CONTROLLER);
  public static final CommandXboxController driver =
      new CommandXboxController(DEVICES.DRIVE_XBOX_CONTROLLER);

  public static void configureBindings(
      SwerveDrive swerveDrive,
      Elevator elevator,
      Manipulator manipulator,
      Hang hang,
      AutonomousCommands autonomous,
      PieceCombos pieceCombos) {

    // Driver
    // Move swerve chassis
    // Rotate Swerve Chassis
    // ^^^ see xbox swerve.java

    // Button to move to processor
    // Button to move to source
    // Auto orient towards algae
    // Button to move to left/right reef (dpad left right)
    // Button for aligning to algae on the reef (dpad up)

    driver.a().whileTrue(autonomous.alignToClosestFace(false));
    driver
    .b()
    .whileTrue(
        autonomous.alignToClosestPoleTeleop(
            AutonomousCommands.PolePattern.RIGHT, () -> rumbleBoth().repeatedly())
            .beforeStarting(() -> LEDs.setState(LEDs.State.AUTO_ALIGN)) 
            .andThen(() -> LEDs.setState(LEDs.State.DEFAULT)) 
    );

    driver
    .x()
    .whileTrue(
        autonomous.alignToClosestPoleTeleop(
            AutonomousCommands.PolePattern.RIGHT, () -> rumbleBoth().repeatedly())
            .beforeStarting(() -> LEDs.setState(LEDs.State.AUTO_ALIGN)) 
            .andThen(() -> LEDs.setState(LEDs.State.DEFAULT)) 
    );
    driver.y();
    driver.start().onTrue(pieceCombos.stow());
    driver.back().whileTrue(swerveDrive.park());
    driver.leftBumper();
    driver.rightBumper();
    driver.rightStick().onTrue(pieceCombos.pickupGroundAlgae());
    // driver.leftStick().onTrue(pieceCombos.algaeProcessor());
    driver
        .leftStick()
        .onTrue(pieceCombos.algaeProcessor()); // TODO: Change to whileTrue() before test
    driver.povCenter(); // USED
    driver.povUp(); // USED
    driver.povDown(); // USED
    driver.povLeft(); // USED
    driver.povRight(); // USED
    driver.leftTrigger(); // USED
    driver.rightTrigger(); // USED
    swerveDrive.setDefaultCommand(new XBoxSwerve(swerveDrive, driver.getHID()));

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

    operator.a().onTrue(pieceCombos.coralL1());
    operator.b().onTrue(pieceCombos.coralL2());
    operator.x().onTrue(pieceCombos.coralL3());
    operator.y().onTrue(pieceCombos.coralL4());

    // operator.a().onTrue(elevator.coralL1());
    // operator.b().onTrue(elevator.coralL2());
    // operator.x().onTrue(elevator.coralL3());
    // operator.y().onTrue(elevator.coralL4());

    // operator.y().onTrue(manipulator.pivot.safe().andThen(elevator.coralL4().alongWith(manipulator.pivot.coralL4())));

    // operator.y().onTrue(elevator.algaeBarge().andThen(manipulator.pivot.algaeBarge()));
    // operator.a().onTrue(manipulator.pivot.coralL1());
    // operator.b().onTrue(manipulator.pivot.coralL23());
    // operator.x().onTrue(manipulator.pivot.coralL4());
    // operator.y().onTrue(manipulator.pivot.stow());
    // operator.start().onTrue(pieceCombos.stow()); // assume this is processor height
    // operator.back().onTrue(elevator.algaeGround());
    // operator.leftStick().onTrue(elevator.algaeL2());
    // operator.rightStick().onTrue(elevator.algaeL3());

    operator.povUp().whileTrue(elevator.up());
    operator.povDown().whileTrue(elevator.down());
    operator.povRight().whileTrue(manipulator.pivot.up());
    operator.povLeft().whileTrue(manipulator.pivot.down());
    operator.back().onTrue(pieceCombos.algaeL3());
    operator.start().onTrue(pieceCombos.algaeL2());
    operator
        .leftStick()
        .onTrue(
            pieceCombos.algaeBargeSetup().andThen(pieceCombos.algaeBargeShoot())); // barge combo
    operator
        .rightStick()
        .onTrue(pieceCombos.intakeCoral().andThen(rumbleBoth())); // big right paddle
        LEDs.setState(LEDs.State.HAS_CORAL);
    operator
      .rightStick()
      .onTrue(
        pieceCombos.intakeCoral()
          .beforeStarting(() -> {
            LEDs.setState(LEDs.State.HAS_CORAL); // ✅ Set LED state when starting
                    rumbleBoth().schedule(); // ✅ Start rumbling
          })
                .andThen(() -> {
                    LEDs.setState(LEDs.State.DEFAULT); // ✅ Reset LED when done
                    rumbleBoth().cancel(); // ✅ Stop rumbling when action ends
                })
        );
    

    operator.rightBumper().whileTrue(manipulator.grabber.adjustCoral()); // intake coral
    operator
        .rightTrigger()
        .whileTrue(
            pieceCombos.intakeAlgaeOrShootCoral().andThen(rumbleBoth())); // drop coral/intake algae
    operator.leftBumper().whileTrue(pieceCombos.algaeBargeShoot()); // shoot barge
    operator.leftTrigger().whileTrue(
    manipulator.grabber.dropAlgae()
        .beforeStarting(() -> LEDs.setState(LEDs.State.GOOD)) // ✅ Only runs when button is pressed
        .andThen(() -> LEDs.setState(LEDs.State.DEFAULT)) // ✅ Resets when button is released
);

    // operator.povUp().onTrue(hang.deploy());
    // operator.povDown().onTrue(hang.hang().onlyIf(() -> DriverStation.getMatchTime() >
    // TIMING.ENDGAME_START.in(Seconds)));
    // operator.rightStick().onTrue(hang.stow());
    // operator.leftBumper().whileTrue(manipulator.algae.action());
    // operator.leftTrigger().whileTrue(manipulator.algae.action());

    // ShuffleboardTab driverTab = DriverDashboard.getTab();

    // driverTab
    //     .addBoolean("Has Coral", () -> manipulator.grabber.hasCoral())
    //     .withWidget(BuiltInWidgets.kBooleanBox)
    //     .withPosition(0, 0)
    //     .withSize(2, 2)
    //     .withProperties(Map.of("min", 0, "max", 100));

    // driverTab
    //     .addBoolean("Has Algae", () -> manipulator.grabber.hasAlgae())
    //     .withWidget(BuiltInWidgets.kBooleanBox)
    //     .withPosition(2, 0)
    //     .withSize(2, 2)
    //     .withProperties(Map.of("min", 0, "max", 100));

    // driverTab.addDouble("Battery Capacity", () -> Constants.SWERVE_DRIVE.BATTERY_VOLTAGE <
    // RobotContainer.getVoltage() ? 100.0 : (RobotContainer.getTotalCurrent() /
    // ((Constants.SWERVE_DRIVE.BATTERY_VOLTAGE - RobotContainer.getVoltage()) /
    // (Constants.SWERVE_DRIVE.BATTERY_RESISTANCE)) * 100.0))
    //   .withWidget(BuiltInWidgets.kDial)
    //   .withPosition(3, 2)
    //   .withSize(2, 2)
    //   .withProperties(Map.of("min", 0, "max", 100));

    // new OperatorFineControls(elevator, manipulator, operator.getHID()).schedule();
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
