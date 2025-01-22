package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLogOutput;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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

  private boolean m_hasSetPose;

  private RAROdometry() {
    super("Odometry");

    m_limelight = new Limelight("limelight");
    m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

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

    m_visionConstants = new VisionConstants(1,100,0,100);
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
    m_gyro.setAngleAdjustment(0.0);
  }

  public AHRS getGyro() {
    return m_gyro;
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

  public void setGyroAngleAdjustment(double angle) {
    m_gyro.setAngleAdjustment(angle);
  }

  public void resetOdometry(Pose2d pose) {
    m_swerve.resetDriveEncoders();

    // We're manually setting the drive encoder positions to 0, since we
    // just reset them, but the encoder isn't reporting 0 yet.
    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_swerve.getKinematics(),
        m_gyro.getRotation2d(),
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
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

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

    if (estimate.pose.getX() <= 0 || estimate.pose.getX() > RobotConstants.robotConfig.Field.k_width) {
      return false;
    }

    if (estimate.pose.getY() <= 0 || estimate.pose.getY() > RobotConstants.robotConfig.Field.k_length) {
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

  //TODO: Add these to Constants when we're done testing them
  private double xyStdDevCoefficient = 0.005;
  private double thetaStdDevCoefficient = 0.01;
  private double stdDevFactor = 0.5; //TODO: Add more!
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

    m_poseEstimator.addVisionMeasurement(
        estimate.pose,
        estimate.timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
  }

  @Override
  public void reset() {
    resetGyro();
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  @Override
  public void periodic() {
    m_poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_swerve.getModule(SwerveDrive.Module.FRONT_LEFT).getPosition(),
            m_swerve.getModule(SwerveDrive.Module.FRONT_RIGHT).getPosition(),
            m_swerve.getModule(SwerveDrive.Module.BACK_RIGHT).getPosition(),
            m_swerve.getModule(SwerveDrive.Module.BACK_LEFT).getPosition()
        });
    PoseEstimate estimate = m_limelight.getPoseEstimation();

    //TODO: I hate this
    if(m_hasSetPose) {
      if(checkPose(estimate)) {
        updatePoseWithStdDev(estimate);
      }

      // if (estimate != null && !isPoseZero(estimate)) {
      //   m_poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
      // }
    } else {
      PoseEstimate megatag1estimate = m_limelight.getMegaTag1PoseEstimation();
      if(megatag1estimate != null && !megatag1estimate.pose.equals(new Pose2d())) {
        m_gyro.setAngleAdjustment(-megatag1estimate.pose.getRotation().getDegrees());
        m_hasSetPose = true;
      }
    }
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void stop() {
    RobotTelemetry.print("Stopping Odometry!");
  }

  // @AutoLogOutput(key = "Odometry/Gyro/YawDeg")
  // public double getGyroYawDeg() {
  //   return m_gyro.getAngle();
  // }

  // @AutoLogOutput(key = "Odometry/Gyro/PitchDeg")
  // public double getGyroPitchDeg() {
  //   return m_gyro.getPitch();
  // }

  // @AutoLogOutput(key = "Odometry/Gyro/RollDeg")
  // public double getGyroRollDeg() {
  //   return m_gyro.getRoll();
  // }

  // @AutoLogOutput(key = "Odometry/Gyro/NavXTimestamp")
  // public double getNavXTimestamp() {
  //   return (double) m_gyro.getLastSensorTimestamp();
  // }

  // @AutoLogOutput(key = "Odometry/Limelight/LimelightPoseEstimation")
  // public Pose2d getLimelightPose2d() {
  //   return m_limelight.getPoseEstimation().pose;
  // }

  @AutoLogOutput(key = "Odometry/PoseEstimator/Pose2d")
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  // @AutoLogOutput(key = "Odometry/PoseEstimator/DistanceMetersFromNearestAprilTag")
  // public double getDistanceMetersFromNearestAprilTag() {
  //   return m_limelight.getPoseEstimation().avgTagDist;
  // }

  // private enum LimelightInstance {
  // LEFT, RIGHT, CENTER
  // }
}
