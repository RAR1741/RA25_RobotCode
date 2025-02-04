package frc.robot.autonomous.tasks;

import frc.robot.RobotTelemetry;

public class DriveTrajectoryTask extends Task {

  @Override
  public void prepare() {
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    boolean finished = true;
    if(finished) {
      RobotTelemetry.print("Drive Trajectory task... complete!");
      return true;
    }
    return false;
  }
}
