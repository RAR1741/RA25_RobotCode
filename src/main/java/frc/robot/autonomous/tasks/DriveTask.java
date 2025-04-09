package frc.robot.autonomous.tasks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotTelemetry;
import frc.robot.subsystems.drivetrain.RAROdometry;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveTask extends Task {
  private final SwerveDrive m_swerve;
  private final RAROdometry m_odometry;
  private double m_targetDistance;
  private double m_xSpeed, m_ySpeed;
  private Pose2d m_startPose;

  private Timer m_runningTimer = new Timer();
  private double m_lastTime = 0;

  public DriveTask(double xSpeed, double ySpeed, double distance) {
    m_swerve = SwerveDrive.getInstance();
    m_odometry = RAROdometry.getInstance();
    m_targetDistance = distance;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
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
    logIsRunning(true);

    m_swerve.drive(m_xSpeed, m_ySpeed, 0, false);
  }

  @Override
  public void updateSim() {
    // This simulates the robot driving in the positive x direction
    if (!RobotBase.isReal()) {
      Pose2d currentPose = m_odometry.getPose();

      // Move "forward", based on the robot's current rotation
      double newX = currentPose.getX()
          + m_xSpeed * (m_runningTimer.get() - m_lastTime) * Math.cos(currentPose.getRotation().getRadians());
      double newY = currentPose.getY()
          + m_ySpeed * (m_runningTimer.get() - m_lastTime) * Math.sin(currentPose.getRotation().getRadians());

      Pose2d newPose = new Pose2d(
          newX,
          newY,
          currentPose.getRotation());

      m_odometry.setPose(newPose);
      m_lastTime = m_runningTimer.get();
    }
  }

  @Override
  public boolean isFinished() {
    Pose2d relativePose = m_startPose.relativeTo(m_odometry.getPose());
    return Math.hypot(relativePose.getX(), relativePose.getY()) >= m_targetDistance;
  }

  @Override
  public void done() {
    logIsRunning(false);
    m_runningTimer.stop();

    RobotTelemetry.print("Auto driving done");
    m_swerve.drive(0, 0, 0, true);
  }
}
