package frc.robot.subsystems;

import java.util.concurrent.locks.ReadWriteLock;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.RAROdometry;

public class Limelight implements Runnable {
  private final NetworkTable m_limelightTable;
  private final LimelightType m_limelightType;
  private VisionConstants m_visionConstants;
  private final String m_limelightName;
  private final Thread m_thread;
  private boolean m_isRunning;
  private int m_internalIMUMode = 0;

  /**
   * Constructor
   */
  public Limelight(String limelightName, ReadWriteLock lock, LimelightType llType) {
    m_limelightName = limelightName;
    m_limelightType = llType;
    m_visionConstants = new VisionConstants(1, 100, 0, 100);

    m_limelightTable = NetworkTableInstance.getDefault().getTable(m_limelightName);
    m_thread = new Thread(this);
    m_thread.setDaemon(true);
  }

  public void start() {
    m_isRunning = true;
    m_thread.start();
  }

  public void stop() {
    m_isRunning = false;
  }

  /**
   * Enable the LEDs
   */
  public void setLightEnabled(boolean enabled) {
    if (m_limelightTable != null) {
      m_limelightTable.getEntry("ledMode").setNumber(enabled ? 3 : 1);
    }
  }

  public void setIMUMode(int mode) {
    m_internalIMUMode = mode;
  }

  /**
   * Get the current bot position
   *
   * @return Current bot pose
   */
  public Pose2d getBotpose2D() {
    return toFieldPose(LimelightHelpers.getBotPose2d(m_limelightName));
  }

  public PoseEstimate getMegaTag1PoseEstimation() {
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);

    if (estimate != null) {
      return estimate;
    }

    return new PoseEstimate();
  }

  /**
   * Converts the limelight coordinate system to the field coordinate system.
   *
   * @param pose Position of the robot
   * @return The position of the robot in terms of the field.
   */
  private Pose2d toFieldPose(Pose2d pose) {
    return pose.relativeTo(new Pose2d(-8.2296, -8.2296 / 2, Rotation2d.fromDegrees(0)));
  }

  public PoseEstimate getPoseEstimation() {
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName);

    if (estimate != null) {
      return estimate;
    }

    return new PoseEstimate();
  }

  public double getLatency() {
    return LimelightHelpers.getLatency_Capture(m_limelightName) + LimelightHelpers.getLatency_Pipeline(m_limelightName);
  }

  public Pose3d getTargetPose_RobotSpace(Pose2d botPose) {
    Pose3d botSpaceTagPose = LimelightHelpers.getTargetPose3d_RobotSpace(m_limelightName);

    return new Pose3d(
        new Translation3d(
            botPose.getTranslation().getX() - botSpaceTagPose.getTranslation().getX(),
            0.0 + botPose.getTranslation().getY() + botSpaceTagPose.getTranslation().getZ(),
            0.5),
        new Rotation3d(botPose.getRotation()));
  }

  public boolean getLightEnabled() {
    return m_limelightTable.getEntry("ledMode").getDouble(1.0) == 3;
  }

  Pose2d nullPose = new Pose2d();

  private boolean isEstimateZero(PoseEstimate estimate) {
    return estimate.pose.equals(nullPose);
  }

  private boolean checkPose(PoseEstimate estimate) {
    if (estimate == null) {
      return false;
    }

    if (isEstimateZero(estimate)) {
      return false;
    }

    if (estimate.pose.getX() <= 0 || estimate.pose.getX() > RobotConstants.robotConfig.Field.k_length) {
      return false;
    }

    if (estimate.pose.getY() <= 0 || estimate.pose.getY() > RobotConstants.robotConfig.Field.k_width) {
      return false;
    }

    if (estimate.tagCount <= 0) {
      return false;
    }

    if (Math.abs(RAROdometry.getInstance().getGyro().getRate()) > 720) {
      return false;
    }

    return true;
  }

  // TODO Add these to Constants when we're done testing them
  private double xyStdDevCoefficient = 0.005;
  private double thetaStdDevCoefficient = 0.01;
  private double stdDevFactor = 0.5; // TODO Add more!
  private boolean useVisionRotation = true;

  private void updatePoseWithStdDev(PoseEstimate estimate) {
    double avgDistance = estimate.avgTagDist;
    double xyStdDev = xyStdDevCoefficient
        * Math.pow(avgDistance, 2.0)
        / estimate.tagCount
        * stdDevFactor
        * (DriverStation.isAutonomous() ? m_visionConstants.autoStdDevScale : 1.0);

    double thetaStdDev = useVisionRotation
        ? thetaStdDevCoefficient
            * Math.pow(avgDistance, 2.0)
            / estimate.tagCount
            * stdDevFactor
            * (DriverStation.isAutonomous() ? m_visionConstants.autoStdDevScale : 1.0)
        : Double.POSITIVE_INFINITY;

    RAROdometry.getInstance().addLLPose(estimate, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
  }

  // private void log(double startTime, PoseEstimate estimate) {
  // Logger.recordOutput("Odometry/Limelight/" + m_limelightName +
  // "/LimelightPoseEstimation", estimate.pose);
  // Logger.recordOutput("Odometry/Limelight/" + m_limelightName +
  // "/AverageTagDistance", estimate.avgTagDist);
  // Logger.recordOutput("Odometry/Limelight/" + m_limelightName + "/ThreadTime",
  // Timer.getFPGATimestamp() - startTime);
  // }

  @Override
  public void run() {
    double targetTime = 0.0;
    switch (m_limelightType) {
      case LL4 -> {
        targetTime = 1.0 / 120.0;
      }
      case LL3 -> {
        targetTime = 1.0 / 50.0;
      }
      case LL2P -> {
        targetTime = 1.0 / 25.0;
      }
    }

    while (true) {
      if (DriverStation.isDisabled()) {
        setIMUMode(IMUMode.INTERNAL_OFF);
      }

      LimelightHelpers.SetIMUMode(m_limelightName, m_internalIMUMode);

      // double yaw = LimelightHelpers.getIMUData(m_limelightName).Yaw;

      // if (DriverStation.getAlliance().isPresent()) {
      // yaw += 180.0;
      // }

      LimelightHelpers.SetRobotOrientation(
          m_limelightName,
          -RAROdometry.getInstance().getGyroYawDeg(),
          0, 0, 0, 0, 0);

      double startTime = Timer.getFPGATimestamp();
      PoseEstimate estimate = getPoseEstimation();

      if (checkPose(estimate)) {
        updatePoseWithStdDev(estimate);
      }

      while (Timer.getFPGATimestamp() - startTime < targetTime) {
        try {
          Thread.sleep(0);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }

      if (DriverStation.isEnabled()) {
        setIMUMode(IMUMode.INTERNAL_ON);
      }
      // log(startTime, estimate);
    }
  }

  // @AutoLogOutput(key =
  // "Odometry/Limelight/{m_limelightTable}/DistanceMetersFromNearestAprilTag")
  // public double getDistanceMetersFromNearestAprilTag() {
  // PoseEstimate estimate = getPoseEstimation();

  // if (estimate != null) {
  // return estimate.avgTagDist;
  // }

  // return 0.0;
  // }

  @AutoLogOutput(key = "Odometry/Limelight/Pose")
  public Pose2d getLLPose() {
    return getPoseEstimation().pose;
  }

  @AutoLogOutput(key = "Odometry/Limelight/IMUYaw")
  public double getIMUYaw() {
    return LimelightHelpers.getIMUData(m_limelightName).Yaw;
  }

  public boolean isRunning() {
    return m_isRunning;
  }

  public enum LimelightType {
    LL2P, LL3, LL4
  }

  public interface IMUMode {
    int INTERNAL_OFF = 1;
    int INTERNAL_ON = 2;
  }
}
