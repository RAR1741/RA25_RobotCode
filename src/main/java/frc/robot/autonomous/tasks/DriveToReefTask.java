package frc.robot.autonomous.tasks;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.PoseAligner;
import frc.robot.subsystems.PoseAligner.Branch;
import frc.robot.subsystems.drivetrain.RAROdometry;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveToReefTask extends Task {
  private Pose2d m_currentPose;
  private Pose2d m_goalPose;

  private final SwerveDrive m_swerve;
  private final RAROdometry m_odometry;

  private final double k_translationErrorThreshold = 0.01; // distance (meters)
  private final double k_rotationErrorThreshold = 0.5; // degrees

  private final Branch m_branch;

  public DriveToReefTask(Branch branch) {
    m_swerve = SwerveDrive.getInstance();
    m_odometry = RAROdometry.getInstance();

    m_branch = branch;
  }

  @Override
  public void prepare() {
    m_currentPose = m_odometry.getPose();

    m_goalPose = PoseAligner.getInstance().getAndCalculateTargetPose(m_currentPose, m_branch);
    m_swerve.resetDriveController();
  }

  @Override
  public void update() {
    logIsRunning(true);

    m_currentPose = m_odometry.getPose();

    m_swerve.drive(m_currentPose, m_goalPose);
  }

  @Override
  public boolean isFinished() {
    double translationError = Math.abs(m_currentPose.getTranslation().getDistance(m_goalPose.getTranslation()));
    double rotationError = Math.abs(m_currentPose.getRotation().minus(m_goalPose.getRotation()).getDegrees());

    return translationError <= k_translationErrorThreshold && rotationError <= k_rotationErrorThreshold;
  }

  @Override
  public void done() {
    logIsRunning(false);
    m_swerve.stop();
  }
}
