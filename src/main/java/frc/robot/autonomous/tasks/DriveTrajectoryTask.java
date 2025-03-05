package frc.robot.autonomous.tasks;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Helpers;
import frc.robot.RobotTelemetry;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drivetrain.RARHolonomicDriveController;
import frc.robot.subsystems.drivetrain.RAROdometry;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveTrajectoryTask extends Task {
  private final RAROdometry m_odometry;
  private final SwerveDrive m_swerve;

  private final RARHolonomicDriveController m_driveController;

  private final Optional<Trajectory<SwerveSample>> m_trajectory;

  private final Timer m_timer;

  public DriveTrajectoryTask(String trajectoryName) {
    m_odometry = RAROdometry.getInstance();
    m_swerve = SwerveDrive.getInstance();

    m_timer = new Timer();

    m_trajectory = Choreo.loadTrajectory(trajectoryName); // Cory O's!
    m_trajectory.get().sampleAt(0, !Helpers.isBlueAlliance());

    m_driveController = new RARHolonomicDriveController(
        RobotConstants.robotConfig.Auto.k_translationConstants,
        RobotConstants.robotConfig.Auto.k_rotationConstants,
        RobotConstants.robotConfig.Robot.k_period);
  }

  @Override
  public void prepare() {
    if (!m_trajectory.isPresent()) {
      m_isFinished = true;
      RobotTelemetry.print("Unable to load Choreo trajectory, was NULL");
      m_prepared = true;
      return;
    }

    // Optional<Pose2d> initialPose =
    // m_trajectory.get().getInitialPose(!Helpers.isBlueAlliance());
    // if(initialPose.isPresent()) {
    // m_odometry.reset();
    // m_odometry.setPose(initialPose.get());
    // }

    m_timer.restart();
    m_prepared = true;
  }

  @Override
  public void update() {
    logIsRunning(true);

    Optional<SwerveSample> sample = m_trajectory.get().sampleAt(m_timer.get(), !Helpers.isBlueAlliance());

    if (sample.isPresent()) {
      ChassisSpeeds m_speeds = m_driveController.calculateTrajectorySpeeds(m_odometry.getPose(), sample.get());

      Logger.recordOutput("Auto/DriveTrajectory/Pose", sample.get().getPose());

      if (RobotBase.isSimulation()) {
        m_odometry.setPose(sample.get().getPose());
      }

      m_swerve.drive(m_speeds); // YOLO
    }

    m_isFinished = m_timer.get() > m_trajectory.get().getTotalTime();
  }

  @Override
  public void done() {
    logIsRunning(false);
  }

  @Override
  public boolean isFinished() {
    if (m_isFinished) {
      RobotTelemetry.print("Drive Trajectory task... complete!");
      return true;
    }
    return false;
  }
}
