package frc.robot.autonomous.tasks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotTelemetry;
import frc.robot.subsystems.drivetrain.RAROdometry;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveForwardTask extends Task {
  private final SwerveDrive m_swerve;
  private final RAROdometry m_odometry;
  private double m_targetDistance;
  private double m_speed;
  private Pose2d m_startPose;

  private Timer m_runningTimer = new Timer();

  public DriveForwardTask(double distance, double speed) {
    m_swerve = SwerveDrive.getInstance();
    m_odometry = RAROdometry.getInstance();
    m_targetDistance = distance;
    m_speed = speed;
  }

  @Override
  public void prepare() {
    m_runningTimer.reset();
    m_runningTimer.start();

    m_startPose = m_odometry.getPose();
    m_prepared = true;
  }

  @Override
  public void update() {
    log(true);

    Pose2d currentPose = m_odometry.getPose();

    double xSpeed = m_speed * Math.cos(currentPose.getRotation().getRadians());
    double ySpeed = m_speed * Math.sin(currentPose.getRotation().getRadians());

    m_swerve.drive(xSpeed, ySpeed, 0, true);
  }

  @Override
  public void updateSim() {
  }

  @Override
  public boolean isFinished() {
    Pose2d relativePose = m_startPose.relativeTo(m_odometry.getPose());
    return Math.hypot(relativePose.getX(), relativePose.getY()) >= m_targetDistance;
  }

  @Override
  public void done() {
    log(false);

    RobotTelemetry.print("Auto driving done");
    m_swerve.drive(0, 0, 0, true);
  }
}
