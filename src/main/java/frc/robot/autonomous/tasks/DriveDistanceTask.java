package frc.robot.autonomous.tasks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotTelemetry;
import frc.robot.subsystems.drivetrain.RAROdometry;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveDistanceTask extends Task {
  private final SwerveDrive m_swerve;
  private final RAROdometry m_odometry;
  private final Pose2d m_goalPose;
  private Pose2d m_currentPose;

  private final double k_translationErrorThreshold = 4.0;
  private final double k_rotationErrorThreshold = 1.0; // degrees

  public DriveDistanceTask(double x, double y) {
    m_swerve = SwerveDrive.getInstance();
    m_odometry = RAROdometry.getInstance();

    m_currentPose = m_odometry.getPose();
    m_goalPose = m_currentPose.transformBy(new Transform2d(m_currentPose.getX() + x, m_currentPose.getY() + y, new Rotation2d()));
  }

  public DriveDistanceTask(Rotation2d rotation) {
    m_swerve = SwerveDrive.getInstance();
    m_odometry = RAROdometry.getInstance();

    m_currentPose = m_odometry.getPose();
    m_goalPose = new Pose2d(m_currentPose.getX(), m_currentPose.getY(), rotation);
  }

  @Override
  public void prepare() {
    m_prepared = true;
  }

  @Override
  public void update() {
    logIsRunning(true);
    m_currentPose = m_odometry.getPose();

    m_swerve.drive(m_currentPose, m_goalPose);
  }

  @Override
  public void updateSim() {
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

    RobotTelemetry.print("Auto driving done");
    m_swerve.drive(0, 0, 0, true);
  }
}