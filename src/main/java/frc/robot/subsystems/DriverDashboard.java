package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DriverDashboard {
    public static NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable("Shuffleboard/Driver Dashboard");
    }

    public static ShuffleboardTab getTab() {
        return Shuffleboard.getTab("Driver Dashboard");
    }
}
