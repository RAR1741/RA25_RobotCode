package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotConstants;

public class RobotTelemetry {

  @SuppressWarnings("resource")
  public RobotTelemetry() {
    /* Display the currently running commands on SmartDashboard */
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0 -> {
        Logger.recordMetadata("GitDirty", "All changes committed");
      }
      case 1 -> {
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
      }
      default -> {
        Logger.recordMetadata("GitDirty", "Unknown");
      }
    }

    Logger.recordMetadata("RobotType", RobotConstants.getInstance().getRobotType().name());
    Logger.recordMetadata("ProjectName", "2025 REEFSCAPE");

    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

    Logger.start();
  }

  public static void print(String output) {
    System.out.println(
        String.format("%.3f", Timer.getFPGATimestamp()) + " || " + output);
  }
}
