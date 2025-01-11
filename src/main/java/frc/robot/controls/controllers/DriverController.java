package frc.robot.controls.controllers;

public class DriverController extends FilteredController {
  public double k_triggerActivationThreshold = 0.5;

  public DriverController(int port) {
    super(port, false, false);
  }

  public DriverController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  // Drive
  public double getForwardAxis() {
    // return -this.getFilteredAxis(Axis.LEFT_Y_AXIS);
    return -this.getFilteredAxis(Axis.LEFT_Y_AXIS) * k_allianceMultiplier;
  }

  public double getStrafeAxis() {
    // return -this.getFilteredAxis(Axis.LEFT_X_AXIS);
    return -this.getFilteredAxis(Axis.LEFT_X_AXIS) * k_allianceMultiplier;
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
}
