package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drivetrain.RAROdometry;

public class PoseAligner extends Subsystem {
  private static PoseAligner m_poseAligner;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private PoseTarget m_target;
  private final PeriodicIO m_periodicIO = new PeriodicIO();

  private PoseAligner() {
    super("PoseAligner");

    m_poseEstimator = RAROdometry.getInstance().getPoseEstimator();
  }

  public static PoseAligner getInstance() {
    if (m_poseAligner == null) {
      m_poseAligner = new PoseAligner();
    }

    return m_poseAligner;
  }

  private static class PeriodicIO {
    Pose2d targetPose = new Pose2d();

    PoseTarget target = PoseTarget.NONE;
  }

  @Override
  public void periodic() {
    // Get the current alliance color
    Alliance alliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
        ? Alliance.Red
        : Alliance.Blue;

    Pose2d allianceReef = alliance == Alliance.Red ? RobotConstants.robotConfig.Field.k_redReefPose
        : RobotConstants.robotConfig.Field.k_blueReefPose;

    Pose2d currentPose = m_poseEstimator.getEstimatedPosition();

    double robotX = currentPose.getX();
    double robotY = currentPose.getY();
    // double robotYaw = currentPose.getRotation().getDegrees();

    double reefX = allianceReef.getX(); // TODO: get a Pose2d corresponding to whichever location we're aligning to
    double reefY = allianceReef.getY();

    // double diagonal = 3.153; // long diagonal of the reef (meters)

    double angle = Units.radiansToDegrees(Math.atan(Math.abs(robotY - reefY) / Math.abs((robotX - reefX))));

    // the robot is "under" the reef relative to the origin point
    if (robotY < reefY) {
      angle = 360 - angle;
    }

    int sector = (int) Math.floor(angle / 60.0);
    int angleOffset = (sector * 60) + 30;

    // if (robotY < reefY + (diagonal / 2)) {
    // angle = 360 - angle;
    // }

    // TODO: set the targetPose
    // m_periodicIO.targetPose = new Pose2d(reefX, reefY,
    // Rotation2d.fromDegrees(angle));

    // m_periodicIO.targetPose = new Pose2d(14.027, 5.645, Rotation2d.fromDegrees(-120)); // april tag id 8
  }

  public Pose2d getTargetPose() {
    return m_periodicIO.targetPose;
  }

  @Override
  public void writePeriodicOutputs() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'writePeriodicOutputs'");
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  public void setTarget(PoseTarget target) {
    m_target = target;
  }

  public enum PoseTarget {
    NONE,
    RED_REEF,
    BLUE_REEF
  }
}
