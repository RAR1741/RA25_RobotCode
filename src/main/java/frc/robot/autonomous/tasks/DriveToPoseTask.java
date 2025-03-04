package frc.robot.autonomous.tasks;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drivetrain.RAROdometry;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveToPoseTask extends Task {
  private Pose2d m_currentPose;
  private final Pose2d m_goalPose;

  private final SwerveDrive m_swerve;
  private final RAROdometry m_odometry;

  private final double k_translationErrorThreshold = 0.0;
  private final double k_rotationErrorThreshold = 0.0;

  public DriveToPoseTask(Pose2d goalPose) {
    m_goalPose = goalPose;

    m_swerve = SwerveDrive.getInstance();
    m_odometry = RAROdometry.getInstance();
  }

  @Override
  public void prepare() {
    m_currentPose = m_odometry.getPose();
  }

  @Override
  public void update() {
    logIsRunning(true);

    m_swerve.drive(m_currentPose, m_goalPose);
  }

  @Override
  public boolean isFinished() {
    logIsRunning(false);

    double translationError = Math.abs(m_currentPose.getTranslation().getDistance(m_goalPose.getTranslation()));
    double rotationError = Math.abs(m_currentPose.getRotation().minus(m_goalPose.getRotation()).getDegrees());

    return translationError <= k_translationErrorThreshold && rotationError <= k_rotationErrorThreshold;
  }

  public void done() {
    m_swerve.stop();
  }
}
