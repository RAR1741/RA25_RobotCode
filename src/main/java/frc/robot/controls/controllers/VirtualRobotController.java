package frc.robot.controls.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.ASPoseHelper;
import frc.robot.subsystems.PoseAligner.Branch;

public class VirtualRobotController extends FilteredController {
  private Pose2d currentPose = new Pose2d();
  private static final double MOVEMENT_SCALE = 0.1; // Scale factor to make movements more manageable

  public VirtualRobotController(int port) {
    super(port, true, true, 0); // Enable deadband and squared input for smoother control
  }

  public VirtualRobotController(int port, boolean useDeadband, boolean useSquaredInput,
      double triggerActivationThreshold) {
    super(port, useDeadband, useSquaredInput, triggerActivationThreshold);
  }

  // Get forward/backward movement (Y translation)
  public double getForwardAxis() {
    return -this.getFilteredAxis(Axis.LEFT_X_AXIS);
  }

  // Get left/right movement (X translation)
  public double getStrafeAxis() {
    return -this.getFilteredAxis(Axis.LEFT_Y_AXIS);
  }

  // Get rotation
  public double getRotationAxis() {
    return -this.getFilteredAxis(Axis.RIGHT_X_AXIS);
  }

  public Branch getWantsAutoPositionBranch() {
    if(this.getRawButtonPressed(Button.LEFT_BUMPER)) {
      return Branch.LEFT;
    } else if (this.getRawButtonPressed(Button.RIGHT_BUMPER)) {
      return Branch.RIGHT;
    }
    return Branch.NONE;
  }

  // Update and return the virtual robot's pose
  public Pose2d updatePose() {
    double forward = getForwardAxis() * MOVEMENT_SCALE;
    double strafe = getStrafeAxis() * MOVEMENT_SCALE;
    double rotation = getRotationAxis() * MOVEMENT_SCALE;

    // Create new pose with updated position and rotation
    currentPose = new Pose2d(
        currentPose.getX() + strafe,
        currentPose.getY() + forward,
        currentPose.getRotation().plus(new Rotation2d(rotation)));

    ASPoseHelper.addPose("VirtualRobot/pose", currentPose);

    return currentPose;
  }

  // Reset pose to origin
  public void resetPose() {
    currentPose = new Pose2d();
  }

  public Pose2d getCurrentPose() {
    return currentPose;
  }
}
