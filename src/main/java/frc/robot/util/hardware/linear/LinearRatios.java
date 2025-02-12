package frc.robot.util.hardware.linear;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Torque;

public class LinearRatios {
  private double rotorToSensorReduction;
  private Distance drumRadius;
  private int stages;
  private double absolutePositionOffset;

  public LinearRatios(double rotorToSensorReduction, Distance drumRadius, int stages) {
    this.rotorToSensorReduction = rotorToSensorReduction;
    this.drumRadius = drumRadius;
    this.stages = stages;
  }

  private Distance rotorToStage(Angle angle, int stage) {
    return drumRadius.times(angle.div(rotorToSensorReduction).in(Radians)).times(stage);
  }

  private double rotorRotationsToStageMeters(int stage) {
    return rotorToStage(Rotations.of(1), stage).in(Meters);
  }

  public Distance rotorToCarriage(Angle angle) {
    return rotorToStage(angle, stages);
  }

  public double rotorRotationsToCarriageMeters() {
    return rotorRotationsToStageMeters(stages);
  }

  public Distance rotorToTop(Angle angle) {
    return rotorToStage(angle, stages - 1);
  }

  public double rotorRotationsToTopMeters() {
    return rotorRotationsToStageMeters(stages - 1);
  }

  private Angle stageToRotor(Distance distance, int stage) {
    return Radians.of(distance.div(drumRadius).div(stage).magnitude())
        .times(rotorToSensorReduction);
  }

  private double stageMetersToRotorRotations(int stage) {
    return stageToRotor(Meters.of(1), stage).in(Rotations);
  }

  public Angle carriageToRotor(Distance distance) {
    return stageToRotor(distance, stages);
  }

  public double carriageMetersToRotorRotations() {
    return stageMetersToRotorRotations(stages);
  }

  public Angle topToRotor(Distance distance) {
    return stageToRotor(distance, stages - 1);
  }

  public double topMetersToRotorRotations() {
    return stageMetersToRotorRotations(stages - 1);
  }

  public Angle rotorToSensor(Angle rotor) {
    return rotor.div(rotorToSensorReduction);
  }

  public double rotorToSensor() {
    return rotorToSensor(Rotations.of(1)).in(Rotations);
  }

  public Angle sensorToRotor(Angle sensor) {
    return sensor.times(rotorToSensorReduction);
  }

  public double sensorToRotor() {
    return sensorToRotor(Rotations.of(1)).in(Rotations);
  }

  private Distance sensorToStage(Angle angle, int stage) {
    return rotorToStage(sensorToRotor(angle), stage);
  }

  public Distance sensorToCarriage(Angle angle) {
    return sensorToStage(angle, stages);
  }

  public double sensorRotationsToCarriageMeters() {
    return sensorToCarriage(Rotations.of(1)).in(Meters);
  }

  public Distance sensorToTop(Angle angle) {
    return sensorToStage(angle, stages - 1);
  }

  public double sensorRotationsToTopMeters() {
    return sensorToTop(Rotations.of(1)).in(Meters);
  }

  private Angle stageToSensor(Distance distance, int stage) {
    return rotorToSensor(stageToRotor(distance, stage));
  }

  public Angle carriageToSensor(Distance distance) {
    return stageToSensor(distance, stages);
  }

  public double carriageMetersToSensorRotations() {
    return carriageToSensor(Meters.of(1)).in(Rotations);
  }

  public Angle topToSensor(Distance distance) {
    return stageToSensor(distance, stages - 1);
  }

  public double topMetersToSensorRotations() {
    return topToSensor(Meters.of(1)).in(Rotations);
  }

  public Force motorToCarriage(Torque motorTorque) {
    return motorTorque.times(rotorToSensorReduction).div(drumRadius).div(stages);
  }

  public int stagesToCarriage() {
    return stages;
  }

  public int stagesToTop() {
    return stages;
  }
}
