package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class ASPoseHelper {
  private static final String LOGGING_KEY = "ASPoseHelper/";

  public static void addRecord(String key_path, double value) {
    Logger.recordOutput(LOGGING_KEY + key_path, value);
  }

  public static void addPose(String key_path, Pose2d pose) {
    Logger.recordOutput(LOGGING_KEY + key_path, pose);
  }

  public static void addPose(String key_path, Pose2d[] pose) {
    Logger.recordOutput(LOGGING_KEY + key_path, pose);
  }

  public static void addPose(String key_path, Pose3d pose) {
    Logger.recordOutput(LOGGING_KEY + key_path, pose);
  }

  public static void addPose(String key_path, Pose3d[] pose) {
    Logger.recordOutput(LOGGING_KEY + key_path, pose);
  }
}
