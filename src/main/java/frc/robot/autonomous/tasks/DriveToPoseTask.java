package frc.robot.autonomous.tasks;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Helpers;
import frc.robot.RobotTelemetry;
import frc.robot.subsystems.PoseAligner;
import frc.robot.subsystems.PoseAligner.Branch;
import frc.robot.subsystems.drivetrain.RAROdometry;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveToPoseTask extends Task {
  private Pose2d m_currentPose;
  private Pose2d m_goalPose;

  private final SwerveDrive m_swerve;
  private final RAROdometry m_odometry;
  private final PoseAligner m_poseAligner;

  private final double k_translationErrorThreshold = 0.01; // distance (meters)
  private final double k_rotationErrorThreshold = 0.5; // degrees

  private final Branch m_branch;
  private final int m_station; // FeederStation

  public DriveToPoseTask(Branch branch) {
    m_swerve = SwerveDrive.getInstance();
    m_odometry = RAROdometry.getInstance();
    m_poseAligner = PoseAligner.getInstance();

    m_branch = branch;
    m_station = -1;
  }

  public DriveToPoseTask(int direction) {
    m_swerve = SwerveDrive.getInstance();
    m_odometry = RAROdometry.getInstance();
    m_poseAligner = PoseAligner.getInstance();

    // go go gadget enable jank
    if (Helpers.isBlueAlliance()) {
      m_station = direction - 4;
    } else {
      m_station = direction - 2;
    }

    m_branch = null;
  }

  @Override
  public void prepare() {
    if (m_station == -1) {
      // we want to target the reef
      m_currentPose = m_odometry.getPose();
      m_goalPose = m_poseAligner.getAndCalculateTargetPose(m_currentPose, m_branch);
    } else if (m_station > 3) {
      RobotTelemetry.print("Bad direction passed to DriveToPoseTask");
    } else {
      // we want to target the feeder station
      Pose2d[] stationPoses = m_poseAligner.getFeederStationPoses();
      m_goalPose = stationPoses[m_station];
    }
    m_swerve.resetDriveController();

    m_prepared = true;
  }

  @Override
  public void update() {
    logIsRunning(true);

    m_currentPose = m_odometry.getPose();

    m_swerve.drive(m_currentPose, m_goalPose);
  }

  @Override
  public boolean isFinished() {
    double translationError = m_currentPose.getTranslation().getDistance(m_goalPose.getTranslation());
    double rotationError = Math.abs(m_currentPose.getRotation().minus(m_goalPose.getRotation()).getDegrees());

    return translationError <= k_translationErrorThreshold && rotationError <= k_rotationErrorThreshold;
  }

  @Override
  public void done() {
    logIsRunning(false);

    m_swerve.stop();
  }
}
