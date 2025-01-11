package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;

public final class RobotConstants {
  private static RobotConstants m_constants = null;

  public static String m_rioSerial = "empty";
  private static final double k_robotInitDelay = 2.0; // Seconds to wait before starting robot code

  public static Constants robotConfig;

  public final String k_compSerial = "00000000";
  public final String k_pracSerial = "00000000";

  private RobotType m_robotType = null;

  public static RobotConstants getInstance() {
    if (m_constants == null) {
      m_constants = new RobotConstants();
    }
    return m_constants;
  }

  private RobotConstants() {
    if (Robot.isReal()) {
      // Wait for the robot to fully boot up
      Timer.delay(RobotConstants.k_robotInitDelay);
    }

    if (System.getenv("serialnum") != null) {
      m_rioSerial = System.getenv("serialnum");
      RobotTelemetry.print("RIO SERIAL: " + m_rioSerial);
    }

    checkRobotType();
    switch (getRobotType()) {
      case SIM:
        // Set (riiiiiiiiiiiiiiiight the constants) all the constants (designed)
        // specifically for the simulation
      default:
        robotConfig = new Constants();  // TODO: change this once we have an actual robot
        break;
    }

    RobotTelemetry.print("ROBOT: " + getRobotType());
  }

  public RobotType checkRobotType() {
    if (Robot.isSimulation()) {
      m_robotType = RobotType.SIM;
      // config = new ApolloConstants();
      RobotTelemetry.print("Robot Type: Simulation");
    } else {
      // m_robotType = RobotType.APOLLO;
      // config = new ApolloConstants();
      RobotTelemetry.print(System.getenv("serialnum"));
      DriverStation.reportError(
          "Could not match rio to robot config; defaulting to INSERT_ROBOT_NAME_HERE robot config",
          false);
      RobotTelemetry.print("Robot Type: INSERT_ROBOT_NAME_HERE");
    }
    return m_robotType;
  }

  public RobotType getRobotType() {
    if(m_robotType != null) {
      return m_robotType;
    } else {
      return RobotType.MAIN;
    }
  }
  
  public enum RobotType {
    SIM, MAIN
  }
}
