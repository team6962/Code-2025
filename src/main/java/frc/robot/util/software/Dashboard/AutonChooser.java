package frc.robot.util.software.Dashboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import java.util.ArrayList;
import java.util.List;

public final class AutonChooser {
  public static ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
  public static List<SimpleWidget> reefOptions = new ArrayList<>();
  public static SimpleWidget rightCoral;
  public static SimpleWidget leftCoral;
  public static SimpleWidget startingAlgae;

  public static boolean startingAlgae() {
    if (tab == null) {
      init();
    }

    return startingAlgae.getEntry().getBoolean(false);
  }

  public static boolean leftCoralStation() {
    if (tab == null) {
      init();
    }

    return leftCoral.getEntry().getBoolean(false);
  }

  public static boolean rightCoralStation() {
    if (tab == null) {
      init();
    }

    return rightCoral.getEntry().getBoolean(false);
  }

  public static List<Integer> reefFaces() {
    if (tab == null) {
      init();
    }

    List<Integer> reefFaces = new ArrayList<>();

    // for (int i = 0; i < Field.REEF_FACES.size(); i++) {
    //   if (reefOptions.get(i).getEntry().getBoolean(false)) {
    //     reefFaces.add(i);
    //   }
    // }
    return reefFaces;
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
            .withPosition(2, 0);
  }
}
