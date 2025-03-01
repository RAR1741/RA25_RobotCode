package frc.robot.autonomous.tasks;

import frc.robot.RobotTelemetry;
import frc.robot.subsystems.drivetrain.RAROdometry;

public class GyroTask extends Task {
  private final RAROdometry m_odometry = RAROdometry.getInstance();

  /**
   * Resets the gyro yaw measurement
   */
  public GyroTask() {
    m_odometry.resetGyro();
  }

  /**
   * Sets the gyro angle adjustment value
   *
   * @param adjustment in degrees (range: -360 to 360)
   */
  public GyroTask(double adjustment) {
    m_odometry.setGyroAngleAdjustment(adjustment);
  }

  @Override
  public void prepare() {
    m_prepared = true;
  }

  @Override
  public void update() {
    logIsRunning(true);
  }

  @Override
  public void done() {
    logIsRunning(false);

    RobotTelemetry.print("Gyro task finished");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
