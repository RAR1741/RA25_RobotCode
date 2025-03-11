package frc.robot.controls.controllers;

import frc.robot.subsystems.PoseAligner.Branch;

public class DriverController extends FilteredController {
  public DriverController(int port) {
    super(port, false, false, 0);
  }

  public DriverController(int port, boolean useDeadband, boolean useSquaredInput, double triggerActivationThreshold) {
    super(port, useDeadband, useSquaredInput, triggerActivationThreshold);
  }

  // Drive
  public double getForwardAxis() {
    return this.getFilteredAxis(Axis.LEFT_Y_AXIS) * m_allianceMultiplier;
  }

  public double getStrafeAxis() {
    return this.getFilteredAxis(Axis.LEFT_X_AXIS) * m_allianceMultiplier;
  }

  public double getTurnAxis() {
    return -this.getFilteredAxis(Axis.RIGHT_X_AXIS);
  }

  public double getSlowScaler() {
    return this.getFilteredAxis(Axis.RIGHT_TRIGGER);
  }

  public double getBoostScaler() {
    return this.getFilteredAxis(Axis.LEFT_TRIGGER);
  }

  // Manual system test modes //
  public double testPositive() {
    return this.getFilteredAxis(Axis.LEFT_TRIGGER);
  }

  public double testNegative() {
    return this.getFilteredAxis(Axis.RIGHT_TRIGGER);
  }

  public boolean getWantsAutoPosition() {
    return getRawButton(Button.X) ||
        getRawButton(Button.LEFT_BUMPER) ||
        getRawButton(Button.RIGHT_BUMPER);
  }

  public Branch getWantsAutoPositionBranch() {
    if (this.getRawButton(Button.LEFT_BUMPER)) {
      return Branch.LEFT;
    } else if (this.getRawButton(Button.RIGHT_BUMPER)) {
      return Branch.RIGHT;
    }
    return Branch.NONE;
  }

  public boolean getWantsAutoPositionPressed() {
    return this.getRawButtonPressed(Button.X);
  }

  public boolean getWantsAutoPositionFeederStation() {
    return this.getRawButtonPressed(Button.B);
  }

  public boolean getWantsResetOdometry() {
    return this.getRawButtonPressed(Button.START);
  }

  public boolean getWantsDeAlgaeTasks() {
    return this.getRawButtonPressed(Button.Y);
  }

  public boolean getWantsClearTellyTasks() {
    return this.getRawButtonReleased(Button.Y);
  }
}
