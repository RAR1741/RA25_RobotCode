package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.RAROdometry;

public class PoseAligner extends Subsystem {
  private static PoseAligner m_poseAligner;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private PoseTarget m_target;

  private PoseAligner() {
    super("PoseAligner");

    m_poseEstimator = RAROdometry.getInstance().getPoseEstimator();
  }

  public PoseAligner getInstance() {
    if (m_poseAligner == null) {
      m_poseAligner = new PoseAligner();
    }

    return m_poseAligner;
  }

  private static class PeriodicIO {
    double elevator_target = 0.0;
    double elevator_power = 0.0;

    boolean is_elevator_pos_control = false;

    PoseTarget target = PoseTarget.NONE;
  }

  @Override
  public void periodic() {
    Pose2d currentPose = m_poseEstimator.getEstimatedPosition();

    double robotX = currentPose.getX();
    double robotY = currentPose.getY();
    // double robotYaw = currentPose.getRotation().getDegrees();

    double reefX = 0.0; // TODO: get a Pose2d corresponding to whichever location we're aligning to
    double reefY = 0.0;

    // double diagonal = 3.153; // long diagonal of the reef (meters)

    double angle = Units.radiansToDegrees(Math.atan(Math.abs(robotY - reefY) / Math.abs((robotX - reefX))));

    // the robot is "under" the reef relative to the origin point
    if (robotY < reefY) {
      angle = 360 - angle;
    }

    // if (robotY < reefY + (diagonal / 2)) {
    // angle = 360 - angle;
    // }
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
