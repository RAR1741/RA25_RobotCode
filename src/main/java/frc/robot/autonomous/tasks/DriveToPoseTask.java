package frc.robot.autonomous.tasks;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.ASPoseHelper;
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

  private final double k_translationErrorThreshold;
  private final double k_rotationErrorThreshold; // degrees

  private Branch m_branch;
  private int m_station; // FeederStation
  private int m_direction = -1;

  public DriveToPoseTask(Branch branch) {
    m_swerve = SwerveDrive.getInstance();
    m_odometry = RAROdometry.getInstance();
    m_poseAligner = PoseAligner.getInstance();

    
    if (branch == Branch.NONE) {
      // Safe pose
      k_translationErrorThreshold = Units.inchesToMeters(4);
      k_rotationErrorThreshold = 1.0;
    } else {
      // Scoring poses
      k_translationErrorThreshold = Units.inchesToMeters(0.5);
      k_rotationErrorThreshold = 0.5;
    }
    

    m_branch = branch;
    m_station = -1;
  }

  public DriveToPoseTask(int direction) {
    m_swerve = SwerveDrive.getInstance();
    m_odometry = RAROdometry.getInstance();
    m_poseAligner = PoseAligner.getInstance();
    m_direction = direction;

    // Feeder poses
    k_translationErrorThreshold = Units.inchesToMeters(12);
    k_rotationErrorThreshold = 5.0;
  }

  @Override
  public void prepare() {
    if (m_direction != -1) {
      // go go gadget enable jank
      if (Helpers.isBlueAlliance()) {
        m_station = m_direction - 4;
      } else {
        m_station = m_direction - 2;
      }

      m_branch = null;
    }

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

    Logger.recordOutput("PoseAligner/Station", m_station);
    ASPoseHelper.addPose("PoseAligner/GoalPose", m_goalPose);
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

    m_swerve.drive(0.0, 0.0, 0.0, true);
  }
}
