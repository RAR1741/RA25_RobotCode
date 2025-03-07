package frc.robot.subsystems.drivetrain;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotTelemetry;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.IMUMode;
import frc.robot.subsystems.Limelight.LimelightType;
import frc.robot.subsystems.Subsystem;

public class RAROdometry extends Subsystem {
  private static RAROdometry m_instance;

  private final AHRS m_gyro;
  private final Limelight m_limelight;
  private final SwerveDrive m_swerve = SwerveDrive.getInstance();
  private SwerveDrivePoseEstimator m_poseEstimator;

  /** Lock used for odometry thread. */
  private final ReadWriteLock m_stateLock = new ReentrantReadWriteLock();
  private OdometryThread m_odometryThread;

  private RAROdometry() {
    super("Odometry");

    m_limelight = new Limelight("limelight-front", m_stateLock, LimelightType.LL4);
    m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);

    m_odometryThread = new OdometryThread(m_stateLock);

    Thread.UncaughtExceptionHandler odometryThreadHandler = new Thread.UncaughtExceptionHandler() {
      @Override
      public void uncaughtException(Thread thread, Throwable throwable) {
        System.err.println("Exception in odometry thread: " + throwable.getMessage());
        System.err.println("Stack trace:");
        throwable.printStackTrace();
        // RobotTelemetry.print("Uncaught exception in odometry thread: " + throwable);
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
  }

  public static RAROdometry getInstance() {
    if (m_instance == null) {
      m_instance = new RAROdometry();
    }
    return m_instance;
  }

  /**
   * Calls the NavX reset function, resetting the Z angle to 0
   */
  public void resetGyro() {
    m_gyro.reset();
  }

  public void resetRotation() {
    m_poseEstimator.resetRotation(new Rotation2d());
    m_limelight.setIMUMode(IMUMode.INTERNAL_OFF);
  }

  public void resetRotation(Rotation2d rotation) {
    m_poseEstimator.resetRotation(rotation);
    m_limelight.setIMUMode(IMUMode.INTERNAL_OFF);
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
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      setGyroAngleAdjustment(0.0);
    } else {
      setGyroAngleAdjustment(180.0);
    }

    resetRotation(getRotation2d());
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

  public void addLLPose(PoseEstimate estimate, Vector<N3> stdDevs) {
    try {
      m_stateLock.writeLock().lock();

      m_poseEstimator.addVisionMeasurement(
          estimate.pose,
          estimate.timestampSeconds,
          stdDevs);
    } finally {
      m_stateLock.writeLock().unlock();
    }
  }

  @Override
  public void reset() {
    m_stateLock.writeLock().lock();

    resetGyro();
    resetOdometry(new Pose2d());

    m_stateLock.writeLock().unlock();
  }

  @Override
  public void periodic() {
    if (!m_odometryThread.isRunning()) {
      m_odometryThread.start();
    }

    if (!m_limelight.isRunning()) {
      m_limelight.start();
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
    return m_gyro.getRotation2d().getDegrees();
  }

  // @AutoLogOutput(key = "Odometry/Gyro/UpdateRate")
  // public double getGyroUpdateRate() {
  // return m_gyro.getActualUpdateRate();
  // }

  @AutoLogOutput(key = "Odometry/Gyro/UpdateCount")
  public double getGyroUpdateCount() {
    return m_gyro.getUpdateCount();
  }

  @AutoLogOutput(key = "Odometry/Gyro/PitchDeg")
  public double getGyroPitchDeg() {
    return m_gyro.getPitch();
  }

  // @AutoLogOutput(key = "Odometry/Gyro/RollDeg")
  // public double getGyroRollDeg() {
  // return m_gyro.getRoll();
  // }

  // @AutoLogOutput(key = "Odometry/Gyro/NavXTimestamp")
  // public double getNavXTimestamp() {
  // return (double) m_gyro.getLastSensorTimestamp();
  // }

  @AutoLogOutput(key = "Odometry/PoseEstimator/Pose2d")
  public Pose2d getPose() {
    Pose2d pose = m_poseEstimator.getEstimatedPosition();

    if (pose != null) {
      return pose;
    }

    return new Pose2d();
  }

  // private enum LimelightInstance {
  // LEFT, RIGHT, CENTER
  // }
}
