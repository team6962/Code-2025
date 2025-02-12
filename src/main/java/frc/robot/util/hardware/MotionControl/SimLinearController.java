package frc.robot.util.hardware.MotionControl;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimLinearController extends SubsystemBase implements LinearController {
    private Distance currentHeight, minHeight, maxHeight, tolerance, targetHeight;
    private DoubleSupplier outputDutyCycle = () -> 0.0;
    private BooleanSupplier doneMoving = () -> false;
    private Timer deltaTimer = new Timer();

    public SimLinearController(
      String name,
      Distance minHeight,
      Distance maxHeight,
      Distance tolerance) {
        setName(name);

        this.minHeight = minHeight;
        this.maxHeight = maxHeight;
        this.tolerance = tolerance;

        this.minHeight = minHeight;
        this.maxHeight = maxHeight;
        this.tolerance = tolerance;

        currentHeight = minHeight;
        targetHeight = currentHeight;

        Logger.logMeasure(this.getName() + "/targetHeight", () -> getTargetHeight());
        Logger.logMeasure(this.getName() + "/measuredHeight", () -> getMeasuredHeight());
    }

    @Override
    public void setTargetHeight(Distance height) {
        outputDutyCycle = () -> height.gt(currentHeight) ? 1.0 : -1.0;
        doneMoving = () -> currentHeight.minus(height).abs(Meters) < tolerance.in(Meters);
        targetHeight = height;
    }

    @Override
    public Distance getTargetHeight() {
        return targetHeight;
    }

    @Override
    public boolean isHeightAchievable(Distance height) {
        return height.gte(minHeight) && height.lte(maxHeight);
    }

    @Override
    public Distance getMeasuredHeight() {
        return currentHeight;
    }

    @Override
    public void moveUp() {
        outputDutyCycle = () -> 1.0;
        doneMoving = () -> false;
    }

    @Override
    public void moveDown() {
        outputDutyCycle = () -> -1.0;
        doneMoving = () -> false;
    }

    @Override
    public boolean doneMoving() {
        return doneMoving.getAsBoolean();
    }

    @Override
    public void stopMotors() {
        outputDutyCycle = () -> 0.0;
        doneMoving = () -> true;
    }

    @Override
    public void periodic() {
        deltaTimer.stop();

        Time deltaTime = Seconds.of(deltaTimer.get());

        deltaTimer.reset();
        deltaTimer.start();

        LinearVelocity velocity = MetersPerSecond.of(1).times(outputDutyCycle.getAsDouble());
        currentHeight = currentHeight.plus(velocity.times(deltaTime));

        currentHeight = MeasureMath.clamp(currentHeight, minHeight, maxHeight);
    }
}
