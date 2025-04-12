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
import frc.robot.subsystems.Elevator.ElevatorState;

public class PoseAligner extends Subsystem {
  private static PoseAligner m_poseAligner;

  private final PeriodicIO m_periodicIO = new PeriodicIO();
  private int m_reefSide = -1;

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
    ElevatorState elevatorState = ElevatorState.STOW;
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
    m_reefSide = reefSide;
    ASPoseHelper.addRecord("ReefSide", reefSide);
    ASPoseHelper.addRecord("CorrectedAngle", correctedAngle);

    Pose2d safePose = poses[reefSide];
    Pose2d scoringPose = getReefScoringPose(safePose, reefSide, branch);

    ElevatorState targetState = Elevator.getInstance().getTargetState();
    if (targetState == ElevatorState.L1 || targetState == ElevatorState.L2 || targetState == ElevatorState.L3) {
      Translation2d translation = new Translation2d(RobotConstants.robotConfig.AutoAlign.k_otherScoringOffset, 0.0);

      scoringPose = scoringPose.transformBy(new Transform2d(translation, new Rotation2d()));
    }

    ASPoseHelper.addPose("ScoringPose", scoringPose);

    m_periodicIO.safePose = safePose;
    m_periodicIO.scoringPose = scoringPose;
  }

  public Pose2d getAndCalculateTargetPose(Pose2d currentPose, Branch branch) {
    calculate(currentPose, branch);

    if (branch == Branch.NONE) {
      return getSafePose();
    }

    if (branch == Branch.ALGAE_REVERSE) {
      return getAlgaeReversePose();
    }

    return getReefScoringPose();
  }

  public Pose2d getSafePose() {
    return m_periodicIO.safePose;
  }

  public Pose2d getReefScoringPose() {
    return m_periodicIO.scoringPose;
  }

  public Pose2d getAlgaeReversePose() {
    Pose2d safePose = m_periodicIO.safePose;
    return safePose.transformBy(new Transform2d(
        RobotConstants.robotConfig.AutoAlign.k_algaeReverseExtraDistance, 0.0, new Rotation2d()));
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

  public void setDesiredElevatorState(ElevatorState elevatorState) {
    m_periodicIO.elevatorState = elevatorState;
  }

  public ElevatorState getDeAlgaeElevatorState() {
    if (Helpers.isBlueAlliance()) {
      return m_reefSide % 2 != 0 ? ElevatorState.ALGAE_HIGH : ElevatorState.ALGAE_LOW;
    } else {
      return m_reefSide % 2 == 0 ? ElevatorState.ALGAE_HIGH : ElevatorState.ALGAE_LOW;
    }
  }

  public Pose2d getReefScoringPose(Pose2d safePose, int reefSide, Branch branch) {
    // x-translation -> down the long side of the field
    double scoringDistance = RobotConstants.robotConfig.AutoAlign.k_l4ScoringDistance;

    // y-translation -> along the shorter side of the field
    double offset = 0.0;

    if (branch == Branch.ALGAE) {
      offset = RobotConstants.robotConfig.AutoAlign.k_algaeHorizontalOffset;
      scoringDistance = RobotConstants.robotConfig.AutoAlign.k_algaeDistance;
    } else {
      if (m_periodicIO.elevatorState == ElevatorState.L1) {
        if (branch == Branch.RIGHT) {
          offset = -RobotConstants.robotConfig.AutoAlign.k_scoringTroughHorizontalOffset;
        } else if (branch == Branch.LEFT) {
          offset = RobotConstants.robotConfig.AutoAlign.k_scoringTroughHorizontalOffset;
        }
      } else {
        if (branch == Branch.RIGHT) {
          offset = -RobotConstants.robotConfig.AutoAlign.k_scoringHorizontalOffset;
        } else if (branch == Branch.LEFT) {
          offset = RobotConstants.robotConfig.AutoAlign.k_scoringHorizontalOffset;
        }
      }
    }

    Translation2d translation = new Translation2d(scoringDistance, offset);

    Pose2d scoringPose = safePose.transformBy(new Transform2d(translation, new Rotation2d()));

    return scoringPose;
  }

  public Pose2d getFeederStationTargetPose(Pose2d currentPose) {
    int direction;
    int allianceOffset = Helpers.isBlueAlliance() ? 0 : 2;

    boolean isLeft = currentPose.getY() > RobotConstants.robotConfig.Field.k_width / 2;
    isLeft = Helpers.isBlueAlliance() ? isLeft : !isLeft;

    // Left is 0, right is 1
    if (isLeft) {
      direction = 0;
    } else {
      direction = 1;
    }

    Pose2d[] feederStationPoses = getFeederStationPoses();

    return feederStationPoses[allianceOffset + direction];
  }

  public Pose2d getBargeTargetPose(Pose2d currentPose) {
    int direction;
    int allianceOffset = Helpers.isBlueAlliance() ? 0 : 2;

    boolean isLeft = currentPose.getY() > RobotConstants.robotConfig.Field.k_width / 2;
    isLeft = Helpers.isBlueAlliance() ? isLeft : !isLeft;

    // Left is 0, right is 1
    if (isLeft) {
      direction = 0;
    } else {
      direction = 1;
    }

    Pose2d[] bargePoses = getBargePoses();

    return bargePoses[allianceOffset + direction];
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

    poses[ReefStartingPoses.CLOSE] = new Pose2d(reefX + offset, reefY, Rotation2d.fromDegrees(180));

    poses[ReefStartingPoses.CLOSE_RIGHT] = new Pose2d(reefX + offset * 0.5, reefY + offset * 0.866,
        Rotation2d.fromDegrees(240));

    poses[ReefStartingPoses.FAR_RIGHT] = new Pose2d(reefX - offset * 0.5, reefY + offset * 0.866,
        Rotation2d.fromDegrees(300));

    poses[ReefStartingPoses.FAR] = new Pose2d(reefX - offset, reefY, Rotation2d.fromDegrees(0));

    poses[ReefStartingPoses.FAR_LEFT] = new Pose2d(reefX - offset * 0.5, reefY - offset * 0.866,
        Rotation2d.fromDegrees(60));

    poses[ReefStartingPoses.CLOSE_LEFT] = new Pose2d(reefX + offset * 0.5, reefY - offset * 0.866,
        Rotation2d.fromDegrees(120));

    return poses;
  }

  public Pose2d[] getBargePoses() {
    Pose2d[] poses = new Pose2d[4];

    double fieldWidth = RobotConstants.robotConfig.Field.k_width;
    double fieldLength = RobotConstants.robotConfig.Field.k_length;

    double xOffset = RobotConstants.robotConfig.AutoAlign.k_bargeXOffset;
    double yOffset = RobotConstants.robotConfig.AutoAlign.k_bargeYOffset;
    double rotOffset = RobotConstants.robotConfig.AutoAlign.k_bargeRotationOffset;

    poses[Barge.BLUE_FAR] = new Pose2d(xOffset, fieldWidth - yOffset, Rotation2d.fromDegrees(-rotOffset));

    poses[Barge.RED_NEAR] = new Pose2d(xOffset, yOffset, Rotation2d.fromDegrees(rotOffset));

    poses[Barge.RED_FAR] = new Pose2d(fieldLength - xOffset, yOffset,
        Rotation2d.fromDegrees(180 - rotOffset));

    poses[Barge.BLUE_NEAR] = new Pose2d(fieldLength - xOffset, fieldWidth - yOffset,
        Rotation2d.fromDegrees(rotOffset - 180));

    ASPoseHelper.addPose("PoseAligner/BargePoses", poses);

    return poses;
  }

  public Pose2d[] getFeederStationPoses() {
    Pose2d[] poses = new Pose2d[4];

    double fieldWidth = RobotConstants.robotConfig.Field.k_width;
    double fieldLength = RobotConstants.robotConfig.Field.k_length;

    double xOffset = RobotConstants.robotConfig.AutoAlign.k_feederStationXOffset;
    double yOffset = RobotConstants.robotConfig.AutoAlign.k_feederStationYOffset;
    double rotOffset = RobotConstants.robotConfig.AutoAlign.k_feederStationRotationOffset;

    poses[FeederStation.BLUE_LEFT] = new Pose2d(xOffset, fieldWidth - yOffset, Rotation2d.fromDegrees(-rotOffset));

    poses[FeederStation.BLUE_RIGHT] = new Pose2d(xOffset, yOffset, Rotation2d.fromDegrees(rotOffset));

    poses[FeederStation.RED_LEFT] = new Pose2d(fieldLength - xOffset, yOffset,
        Rotation2d.fromDegrees(180 - rotOffset));

    poses[FeederStation.RED_RIGHT] = new Pose2d(fieldLength - xOffset, fieldWidth - yOffset,
        Rotation2d.fromDegrees(rotOffset - 180));

    ASPoseHelper.addPose("PoseAligner/FeederStationPoses", poses);

    return poses;
  }

  public interface ReefStartingPoses {
    int CLOSE = 0; // high
    int CLOSE_RIGHT = 1; // low
    int FAR_RIGHT = 2; // high
    int FAR = 3; // low
    int FAR_LEFT = 4; // high
    int CLOSE_LEFT = 5; // low
  }

  public enum Branch {
    LEFT,
    RIGHT,
    ALGAE,
    ALGAE_REVERSE,
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

  public interface Barge {
    int BLUE_NEAR = 0;
    int BLUE_FAR = 1;
    int RED_NEAR = 2;
    int RED_FAR = 3;
    int NEAR = 4;
    int FAR = 5;
  }
}
