package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.ASPoseHelper;
import frc.robot.Helpers;
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
  }

  public void calculate(Pose2d currentPose) {
    // Get the current alliance color
    Alliance alliance = DriverStation.getAlliance().isPresent() &&
        DriverStation.getAlliance().get() == Alliance.Red
            ? Alliance.Red
            : Alliance.Blue;

    ASPoseHelper.addPose("Red/Reef/pose", RobotConstants.robotConfig.Field.k_redReefPose);
    ASPoseHelper.addPose("Blue/Reef/pose", RobotConstants.robotConfig.Field.k_blueReefPose);

    Pose3d allianceReef = alliance == Alliance.Red ? RobotConstants.robotConfig.Field.k_redReefPose
        : RobotConstants.robotConfig.Field.k_blueReefPose;

    ASPoseHelper.addPose("TargetAngle", new Pose2d[] { currentPose, allianceReef.toPose2d() });

    double baseAngle = Math.toDegrees(Math.atan2(
        allianceReef.getY() - currentPose.getY(), allianceReef.getX() - currentPose.getX()));

    // 180 to face the red wall, 30 to account for the reef's orientation
    int correctedAngle = (int) Helpers.modDegrees(baseAngle + 180.0 + 30.0);

    // TODO: we only need to do this once/when the alliance changes
    Pose2d[] poses = getAllianceReefScoringPoses(allianceReef);
    String loggingKey = (alliance == Alliance.Red ? "Red/Reef/targets" : "Blue/Reef/targets");
    ASPoseHelper.addPose(loggingKey, poses);

    m_periodicIO.targetPose = poses[Math.floorDiv(correctedAngle, 60)];
  }

  public Pose2d getAndCalculateTargetPose(Pose2d currentPose) {
    calculate(currentPose);
    return getTargetPose();
  }

  public Pose2d getTargetPose() {
    return m_periodicIO.targetPose;
  }

  @Override
  public void writePeriodicOutputs() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method
    // 'writePeriodicOutputs'");
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  public void setTarget(PoseTarget target) {
    m_target = target;
  }

  public enum PoseTarget {
    NONE,
    RED_REEF,
    BLUE_REEF
  }

  /**
   * Generates a new Pose2d for scoring on all 6 sides of the given alliance reef.
   *
   * @param allianceReefPose The Pose2d object representing the alliance reef
   *                         location.
   * @return An array of Pose2d objects representing scoring poses around the
   *         alliance reef.
   */
  public Pose2d[] getAllianceReefScoringPoses(Pose3d allianceReefPose) {
    Pose2d[] poses = new Pose2d[6];

    // Calculate poses for each side of the reef (assuming hexagonal reef)
    // You might need to adjust these offsets based on the actual reef dimensions
    // and robot's approach
    double reefX = allianceReefPose.getX();
    double reefY = allianceReefPose.getY();

    double offset = 1.5; // Offset from the reef center, adjust as needed

    poses[0] = new Pose2d(reefX + offset, reefY, Rotation2d.fromDegrees(180)); // Right side
    poses[1] = new Pose2d(reefX + offset * 0.5, reefY + offset * 0.866, Rotation2d.fromDegrees(240)); // Top-right side
    poses[2] = new Pose2d(reefX - offset * 0.5, reefY + offset * 0.866, Rotation2d.fromDegrees(300)); // Top-left side
    poses[3] = new Pose2d(reefX - offset, reefY, Rotation2d.fromDegrees(0)); // Left side
    poses[4] = new Pose2d(reefX - offset * 0.5, reefY - offset * 0.866, Rotation2d.fromDegrees(60)); // Bottom-left side
    poses[5] = new Pose2d(reefX + offset * 0.5, reefY - offset * 0.866, Rotation2d.fromDegrees(120)); // Bottom-right
                                                                                                      // side

    return poses;
  }
}
