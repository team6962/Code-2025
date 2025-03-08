package frc.robot.util.software.Dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.subsystems.DriverDashboard;
import java.util.ArrayList;
import java.util.List;

public final class AutonChooser {
  public static NetworkTable table =
      NetworkTableInstance.getDefault().getTable("Shuffleboard/Autonomous");
  public static ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
  public static List<SimpleWidget> reefOptions = new ArrayList<>();
  public static SimpleWidget rightCoral;
  public static SimpleWidget leftCoral;
  public static SimpleWidget startingAlgae;
  public static SimpleWidget autoComputed;

  public static boolean startingAlgae() {
    return table.getEntry("Starting Algae").getBoolean(false);
    // return startingAlgae.getEntry().getBoolean(false);
  }

  public static boolean leftCoralStation() {
    return table.getEntry("Left Station").getBoolean(false);
    // return leftCoral.getEntry().getBoolean(false);
  }

  public static boolean rightCoralStation() {
    return table.getEntry("Right Station").getBoolean(false);
    // return rightCoral.getEntry().getBoolean(false);
  }

  public static List<Integer> reefFaces() {
    List<Integer> reefFaces = new ArrayList<>();

    for (int i = 0; i < 6; i++) {
      if (table.getEntry("Face " + (i + 1)).getBoolean(false)) {
        reefFaces.add(i);
      }
      // if (reefOptions.get(i).getEntry().getBoolean(false)) {
      //   reefFaces.add(i);
      // }
    }

    return reefFaces;
  }

  public static void setAutoComputed(boolean value) {
    DriverDashboard.getTable().getEntry("Auto Precomputed").setBoolean(value);
  }

  public static void init() {
    reefOptions.add(
        tab.add("Face " + 1, false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .withPosition(1, 3));
    reefOptions.add(
        tab.add("Face " + 2, false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .withPosition(0, 2));
    reefOptions.add(
        tab.add("Face " + 3, false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .withPosition(0, 1));
    reefOptions.add(
        tab.add("Face " + 4, false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .withPosition(1, 0));
    reefOptions.add(
        tab.add("Face " + 5, false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .withPosition(2, 1));
    reefOptions.add(
        tab.add("Face " + 6, false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .withPosition(2, 2));

    rightCoral =
        tab.add("Right Station", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .withPosition(0, 3);
    leftCoral =
        tab.add("Left Station", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .withPosition(2, 3);
    startingAlgae =
        tab.add("Starting Algae", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .withPosition(4, 2);
    autoComputed =
        DriverDashboard.getTab()
            .add("Auto Precomputed", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(2, 2)
            .withPosition(4, 0);
  }
}
