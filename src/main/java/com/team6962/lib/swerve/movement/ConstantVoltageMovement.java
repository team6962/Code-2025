package com.team6962.lib.swerve.movement;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.team6962.lib.swerve.SwerveCore;
import com.team6962.lib.swerve.module.SwerveModule;

import edu.wpi.first.units.measure.Voltage;

public class ConstantVoltageMovement implements SwerveMovement {
    private Voltage steerVoltage = Volts.of(0);
    private Voltage driveVoltage = Volts.of(0);
    private boolean singleModule = false;

    public ConstantVoltageMovement withSteerVoltage(Voltage voltage) {
      this.steerVoltage = voltage;
      return this;
    }

    public ConstantVoltageMovement withDriveVoltage(Voltage voltage) {
      this.driveVoltage = voltage;
      return this;
    }

    public ConstantVoltageMovement withSingleModule(boolean singleModule) {
      this.singleModule = singleModule;
      return this;
    }

    @Override
    public void execute(SwerveCore drivetrain) {
      for (SwerveModule module : drivetrain.getModules()) {
        if (singleModule && module.getModuleCorner().index != 0) {
          module.drive(
            new VoltageOut(0),
            new VoltageOut(0)
          );

          continue;
        }

        module.drive(
          new VoltageOut(driveVoltage),
          new VoltageOut(steerVoltage)
        );
      }
    }

    @Override
    public SwerveMovement cleared() {
      return null;
    }
}
