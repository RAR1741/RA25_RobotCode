package frc.robot.subsystems.drivetrain;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotTelemetry;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Subsystem;

public class RAROdometry extends Subsystem {
  private static RAROdometry m_instance;

  private final AHRS m_gyro;
  private final Limelight m_limelight;

  private final SwerveDrive m_swerve = SwerveDrive.getInstance();

  private VisionConstants m_visionConstants;

  private SwerveDrivePoseEstimator m_poseEstimator;

  /** Lock used for odometry thread. */
  private final ReadWriteLock m_stateLock = new ReentrantReadWriteLock();

  private OdometryThread m_odometryThread;

  private RAROdometry() {
    super("Odometry");

    m_limelight = new Limelight("limelight");
    m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);

    m_odometryThread = new OdometryThread(m_stateLock);

    Thread.UncaughtExceptionHandler odometryThreadHandler = new Thread.UncaughtExceptionHandler() {
      @Override
      public void uncaughtException(Thread thread, Throwable throwable) {
        RobotTelemetry.print("Uncaught exception in odometry thread: " + throwable);
      }
    };

    Thread.setDefaultUncaughtExceptionHandler(odometryThreadHandler);

    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_swerve.getKinematics(),
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_swerve.getModule(SwerveDrive.Module.FRONT_LEFT).getPosition(),
            m_swerve.getModule(SwerveDrive.Module.FRONT_RIGHT).getPosition(),
            m_swerve.getModule(SwerveDrive.Module.BACK_LEFT).getPosition(),
            m_swerve.getModule(SwerveDrive.Module.BACK_RIGHT).getPosition()
        },
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)) // we clarified this works
    );

    // m_visionConstants = new VisionConstants(1,100,0,100);
  }

  public static RAROdometry getInstance() {
    if (m_instance == null) {
      m_instance = new RAROdometry();
    }
    return m_instance;
  }

  /**
   * Calls the NavX reset function, resetting the Z angle to 0
   * ur gae mccabe
   */
  public void resetGyro() {
    m_gyro.reset();
  }

  public AHRS getGyro() {
    return m_gyro;
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return m_poseEstimator;
  }

  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_swerve.getModule(SwerveDrive.Module.FRONT_LEFT).getPosition(),
            m_swerve.getModule(SwerveDrive.Module.FRONT_RIGHT).getPosition(),
            m_swerve.getModule(SwerveDrive.Module.BACK_LEFT).getPosition(),
            m_swerve.getModule(SwerveDrive.Module.BACK_RIGHT).getPosition()
        },
        pose);
  }

  public void setAllianceGyroAngleAdjustment() {
    if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      m_gyro.setAngleAdjustment(180.0);
    } else {
      m_gyro.setAngleAdjustment(0.0);
    }
  }

  public void setGyroAngleAdjustment(double angle) {
    m_gyro.setAngleAdjustment(angle);
  }

  public void resetOdometry(Pose2d pose) {
    m_swerve.resetDriveEncoders();

    // We're manually setting the drive encoder positions to 0, since we
    // just reset them, but the encoder isn't reporting 0 yet.
    m_poseEstimator.resetPosition(new Rotation2d(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_swerve.getModule(SwerveDrive.Module.FRONT_LEFT).getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_swerve.getModule(SwerveDrive.Module.FRONT_RIGHT).getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_swerve.getModule(SwerveDrive.Module.BACK_LEFT).getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_swerve.getModule(SwerveDrive.Module.BACK_RIGHT).getTurnPosition())),
        },
        new Pose2d());

    setPose(pose);
  }

  Pose2d nullPose = new Pose2d();

  private boolean isPoseZero(PoseEstimate estimate) {
    return estimate.pose.equals(nullPose);
  }

  private boolean checkPose(PoseEstimate estimate) {
    if (estimate == null) {
      return false;
    }

    if (isPoseZero(estimate)) {
      return false;
    }

    if (estimate.pose.getX() <= 0 || estimate.pose.getX() > RobotConstants.robotConfig.Field.k_length) {
      return false;
    }

    if (estimate.pose.getY() <= 0 || estimate.pose.getY() > RobotConstants.robotConfig.Field.k_width) {
      return false;
    }

    if (estimate.tagCount <= 0) {
      return false;
    }

    if (Math.abs(m_gyro.getRate()) > 720) {
      return false;
    }

    return true;
  }

  // TODO: Add these to Constants when we're done testing them
  private double xyStdDevCoefficient = 0.005;
  private double thetaStdDevCoefficient = 0.01;
  private double stdDevFactor = 0.5; // TODO: Add more!
  private boolean useVisionRotation = true;

  private void updatePoseWithStdDev(PoseEstimate estimate) {
    double avgDistance = estimate.avgTagDist;
    double xyStdDev = xyStdDevCoefficient
        * Math.pow(avgDistance, 2.0)
        / estimate.tagCount
        * stdDevFactor
        * (DriverStation.isAutonomous() ? m_visionConstants.autoStdDevScale : 1.0);

    double thetaStdDev = useVisionRotation
        ? thetaStdDevCoefficient
            * Math.pow(avgDistance, 2.0)
            / estimate.tagCount
            * stdDevFactor
            * (DriverStation.isAutonomous() ? m_visionConstants.autoStdDevScale : 1.0)
        : Double.POSITIVE_INFINITY;

    try {
      m_stateLock.writeLock().lock();

      m_poseEstimator.addVisionMeasurement(
          estimate.pose,
          estimate.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
    } finally {
      m_stateLock.writeLock().unlock();
    }
  }

  @Override
  public void reset() {
    // m_odometryThread.stop();

    // try {
    // m_stateLock.writeLock().lockInterruptibly();
    // } catch (InterruptedException exception) {
    // exception.printStackTrace();
    // }

    m_stateLock.writeLock().lock();
    // m_stateLock.readLock().lock();

    resetGyro();
    resetOdometry(new Pose2d());

    m_stateLock.writeLock().unlock();
    // m_stateLock.readLock().unlock();

    // m_odometryThread.start();
  }

  @Override
  public void periodic() {
    if (!m_odometryThread.isRunning()) {
      m_odometryThread.start();
    }

    PoseEstimate estimate = m_limelight.getPoseEstimation();

    if (checkPose(estimate)) {
      updatePoseWithStdDev(estimate);
    }

    logAprilTagData();
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void stop() {
    RobotTelemetry.print("Stopping Odometry!");
  }

  private void logAprilTagData() {
    Logger.recordOutput(
        "Odometry/Limelight/M2AprilTag",
        m_limelight.getTargetPose_RobotSpace(getPose()));
  }

  @AutoLogOutput(key = "Odometry/Gyro/YawDeg")
  public double getGyroYawDeg() {
    return m_gyro.getAngle();
  }

  // @AutoLogOutput(key = "Odometry/Gyro/UpdateRate")
  // public double getGyroUpdateRate() {
  //   return m_gyro.getActualUpdateRate();
  // }

  @AutoLogOutput(key = "Odometry/Gyro/UpdateCount")
  public double getGyroUpdateCount() {
    return m_gyro.getUpdateCount();
  }

  @AutoLogOutput(key = "Odometry/Gyro/PitchDeg")
  public double getGyroPitchDeg() {
    return m_gyro.getPitch();
  }

  @AutoLogOutput(key = "Odometry/Gyro/RollDeg")
  public double getGyroRollDeg() {
    return m_gyro.getRoll();
  }

  @AutoLogOutput(key = "Odometry/Gyro/NavXTimestamp")
  public double getNavXTimestamp() {
    return (double) m_gyro.getLastSensorTimestamp();
  }

  @AutoLogOutput(key = "Odometry/Limelight/LimelightPoseEstimation")
  public Pose2d getLimelightPose2d() {
    Pose2d pose = m_limelight.getPoseEstimation().pose;

    if (pose != null) {
      return pose;
    }
    return new Pose2d();
  }

  @AutoLogOutput(key = "Odometry/PoseEstimator/Pose2d")
  public Pose2d getPose() {
    Pose2d pose = m_poseEstimator.getEstimatedPosition();

    if (pose != null) {
      return pose;
    }

    return new Pose2d();
  }

  @AutoLogOutput(key = "Odometry/PoseEstimator/DistanceMetersFromNearestAprilTag")
  public double getDistanceMetersFromNearestAprilTag() {
    PoseEstimate estimate = m_limelight.getPoseEstimation();

    if (estimate != null) {
      return estimate.avgTagDist;
    }

    return 0.0;
  }

  // private enum LimelightInstance {
  // LEFT, RIGHT, CENTER
  // }
}
