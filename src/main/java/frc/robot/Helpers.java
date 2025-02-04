package frc.robot;

import java.util.Optional;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.DriverStation;

public class Helpers {
  public static boolean isBlueAlliance() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    if (DriverStation.getAlliance().isPresent()) {
      return alliance.get() == DriverStation.Alliance.Blue;
    }
    
    return true; // default to blue
  }

  public static double modRotations(double input) {
    input %= 1.0;
    if (input < 0.0) {
      input += 1.0;
    }
    return input;
  }

  public static double modRadians(double input) {
    input %= (2.0 * Math.PI);
    if (input < 0.0) {
      input += (2.0 * Math.PI);
    }
    return input;
  }

  public static double modDegrees(double input) {
    input %= 360.0;
    if (input < 0.0) {
      input += 360.0;
    }
    return input;
  }

  public static double getVoltage(SparkBase motor) {
    return motor.getBusVoltage() * motor.getAppliedOutput();
  }

  /**
   * @param wheelRPS      Wheel Velocity: (in Rotations per Second)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Velocity: (in Meters per Second)
   */
  public static double RPSToMPS(double wheelRPS, double circumference) {
    double wheelMPS = wheelRPS * circumference;
    return wheelMPS;
  }

  /**
   * @param wheelMPS      Wheel Velocity: (in Meters per Second)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Velocity: (in Rotations per Second)
   */
  public static double MPSToRPS(double wheelMPS, double circumference) {
    double wheelRPS = wheelMPS / circumference;
    return wheelRPS;
  }

  /**
   * @param wheelRotations Wheel Position: (in Rotations)
   * @param circumference  Wheel Circumference: (in Meters)
   * @return Wheel Distance: (in Meters)
   */
  public static double rotationsToMeters(double wheelRotations, double circumference) {
    double wheelMeters = wheelRotations * circumference;
    return wheelMeters;
  }

  /**
   * @param wheelMeters   Wheel Distance: (in Meters)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Position: (in Rotations)
   */
  public static double metersToRotations(double wheelMeters, double circumference) {
    double wheelRotations = wheelMeters / circumference;
    return wheelRotations;
  }
}
