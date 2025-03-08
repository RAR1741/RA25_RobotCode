package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.ASPoseHelper;
import frc.robot.Helpers;
import frc.robot.constants.RobotConstants;

public class PoseAligner extends Subsystem {
  private static PoseAligner m_poseAligner;

  private final PeriodicIO m_periodicIO = new PeriodicIO();

  private PoseAligner() {
    super("PoseAligner");
  }

  public static PoseAligner getInstance() {
    if (m_poseAligner == null) {
      m_poseAligner = new PoseAligner();
    }

    return m_poseAligner;
  }

  private static class PeriodicIO {
    Pose2d safePose = new Pose2d();
    Pose2d scoringPose = new Pose2d();
  }

  @Override
  public void periodic() {
    getFeederStationPoses();
  }

  public void calculate(Pose2d currentPose, Branch branch) {
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

    // get the pose corresponding to what sector we're in
    int reefSide = Math.floorDiv(correctedAngle, 60);
    ASPoseHelper.addRecord("ReefSide", reefSide);
    ASPoseHelper.addRecord("CorrectedAngle", correctedAngle);

    Pose2d safePose = poses[reefSide];
    Pose2d scoringPose = getScoringPose(safePose, reefSide, branch);
    ASPoseHelper.addPose("ScoringPose", scoringPose);

    m_periodicIO.safePose = safePose;
    m_periodicIO.scoringPose = scoringPose;
  }

  public Pose2d getAndCalculateTargetPose(Pose2d currentPose, Branch branch) {
    calculate(currentPose, branch);

    if (branch == Branch.NONE) {
      return getSafePose();
    }

    return getScoringPose();
  }

  // public Pose2d getAndCalculateTargetPose(Pose2d currentPose, FeederStation
  // station) {
  // calculate(currentPose, station);
  // }

  public Pose2d getSafePose() {
    return m_periodicIO.safePose;
  }

  public Pose2d getScoringPose() {
    return m_periodicIO.scoringPose;
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void stop() {
  }

  @Override
  public void reset() {
  }

  public Pose2d getScoringPose(Pose2d currentPose, int reefSide, Branch branch) {
    // x-translation -> down the long side of the field
    double scoringDistance = RobotConstants.robotConfig.AutoAlign.k_scoringDistance;

    // y-translation -> along the shorter side of the field
    double scoringHorizontalOffset = RobotConstants.robotConfig.AutoAlign.k_scoringHorizontalOffset;

    if (branch == Branch.RIGHT) {
      scoringHorizontalOffset = -scoringHorizontalOffset;
    }

    Translation2d offset = new Translation2d(scoringDistance, scoringHorizontalOffset);

    Pose2d scoringPose = currentPose.transformBy(new Transform2d(offset, new Rotation2d()));

    return scoringPose;
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

    // Calculate poses for each side of the reef
    // You might need to adjust these offsets based on the actual reef dimensions
    // and robot's approach
    double reefX = allianceReefPose.getX();
    double reefY = allianceReefPose.getY();

    double offset = RobotConstants.robotConfig.AutoAlign.k_minSafeTargetDistance;

    poses[ReefStartingPoses.RIGHT_SIDE] = new Pose2d(reefX + offset, reefY, Rotation2d.fromDegrees(180));

    poses[ReefStartingPoses.TOP_RIGHT_SIDE] = new Pose2d(reefX + offset * 0.5, reefY + offset * 0.866,
        Rotation2d.fromDegrees(240));

    poses[ReefStartingPoses.TOP_LEFT_SIDE] = new Pose2d(reefX - offset * 0.5, reefY + offset * 0.866,
        Rotation2d.fromDegrees(300));

    poses[ReefStartingPoses.LEFT_SIDE] = new Pose2d(reefX - offset, reefY, Rotation2d.fromDegrees(0));

    poses[ReefStartingPoses.BOTTOM_LEFT_SIDE] = new Pose2d(reefX - offset * 0.5, reefY - offset * 0.866,
        Rotation2d.fromDegrees(60));

    poses[ReefStartingPoses.BOTTOM_RIGHT_SIDE] = new Pose2d(reefX + offset * 0.5, reefY - offset * 0.866,
        Rotation2d.fromDegrees(120));

    return poses;
  }

  public Pose2d[] getFeederStationPoses() {
    Pose2d[] poses = new Pose2d[4];

    double fieldWidth = RobotConstants.robotConfig.Field.k_width;
    double fieldLength = RobotConstants.robotConfig.Field.k_length;

    double xOffset = RobotConstants.robotConfig.AutoAlign.k_feederStationXOffset;
    double yOffset = RobotConstants.robotConfig.AutoAlign.k_feederStationYOffset;
    double rotOffset = RobotConstants.robotConfig.AutoAlign.k_feederStationRotationOffset;

    poses[FeederStation.BLUE_RIGHT] = new Pose2d(xOffset, yOffset, Rotation2d.fromDegrees(rotOffset));

    poses[FeederStation.BLUE_LEFT] = new Pose2d(xOffset, fieldWidth - yOffset, Rotation2d.fromDegrees(-rotOffset));

    poses[FeederStation.RED_LEFT] = new Pose2d(fieldLength - xOffset, yOffset,
        Rotation2d.fromDegrees(180 - rotOffset));

    poses[FeederStation.RED_RIGHT] = new Pose2d(fieldLength - xOffset, fieldWidth - yOffset,
        Rotation2d.fromDegrees(rotOffset - 180));

    ASPoseHelper.addPose("PoseAligner/FeederStationPoses", poses);

    return poses;
  }

  public interface ReefStartingPoses {
    int RIGHT_SIDE = 0;
    int TOP_RIGHT_SIDE = 1;
    int TOP_LEFT_SIDE = 2;
    int LEFT_SIDE = 3;
    int BOTTOM_LEFT_SIDE = 4;
    int BOTTOM_RIGHT_SIDE = 5;
  }

  public enum Branch {
    LEFT,
    RIGHT,
    NONE
  }

  public interface FeederStation {
    int BLUE_LEFT = 0;
    int BLUE_RIGHT = 1;
    int RED_LEFT = 2;
    int RED_RIGHT = 3;
    int LEFT = 4;
    int RIGHT = 5;
  }

  // TODO maybe change the starting pose labels to tag-specific for easier
  // labeling
}
