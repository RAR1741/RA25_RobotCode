package frc.robot.autonomous.tasks;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotTelemetry;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class WaitTask extends Task {
  private Timer m_runningTimer = new Timer();
  private double m_targetTime;

  private final WaitCondition m_condition;

  private final SwerveDrive m_swerve = SwerveDrive.getInstance();
  private final EndEffector m_endEffector = EndEffector.getInstance();

  public enum WaitCondition {
    TIME,
    END_EFFECTOR_INDEXED,
    LOW_SPEED
    // CORAL_SCORED
  }

  public WaitTask(double timeSeconds) {
    m_condition = WaitCondition.TIME;
    m_targetTime = timeSeconds;
  }

  public WaitTask(WaitCondition condition) {
    m_condition = condition;
  }

  @Override
  public void prepare() {
    m_runningTimer.start();
    m_prepared = true;
  }

  @Override
  public void update() {
    logIsRunning(true);
  }

  @Override
  public boolean isFinished() {
    switch (m_condition) {
      case TIME -> {
        return m_runningTimer.get() >= m_targetTime;
      }
      case END_EFFECTOR_INDEXED -> {
        return m_endEffector.isSafeToScore();
      }
      case LOW_SPEED -> {
        ChassisSpeeds currentSpeed = m_swerve.getChassisSpeeds();
        return Math.hypot(currentSpeed.vxMetersPerSecond,
            currentSpeed.vyMetersPerSecond) < RobotConstants.robotConfig.Auto.k_lowSpeed;
      }
      // case CORAL_SCORED -> {
      //   return !LaserCanHandler.getInstance().getExitSeesCoral();
      // }
    }
    return true;
  }

  @Override
  public void done() {
    logIsRunning(false);
    m_runningTimer.stop();

    RobotTelemetry.print("Auto wait done");
  }
}
