package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.constants.RobotConstants;
import frc.robot.wrappers.PIDConstants;
import frc.robot.wrappers.ProfiledPIDConstants;

/** Pose targeter for holonomic drive trains */
public class RARHolonomicDriveController {
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController rotationController;

  private boolean isEnabled = true;

  /**
   * Constructs a HolonomicDriveController
   *
   * @param translationConstants PID constants for the translation PID controllers
   * @param rotationConstants    PID constants for the rotation controller
   * @param period               Period of the control loop in seconds
   */
  public RARHolonomicDriveController(
      PIDConstants translationConstants, ProfiledPIDConstants rotationConstants, double period) {
    this.xController = new PIDController(
        translationConstants.k_P, translationConstants.k_I, translationConstants.k_D, period);
    this.xController.setIntegratorRange(-translationConstants.k_iZone, translationConstants.k_iZone);

    this.yController = new PIDController(
        translationConstants.k_P, translationConstants.k_I, translationConstants.k_D, period);
    this.yController.setIntegratorRange(-translationConstants.k_iZone, translationConstants.k_iZone);

    // Temp rate limit of 0, will be changed in calculate
    this.rotationController = new ProfiledPIDController(
        rotationConstants.k_P,
        rotationConstants.k_I,
        rotationConstants.k_D,
        new Constraints(rotationConstants.k_maxVel, rotationConstants.k_maxAcc),
        period);
    this.rotationController.setIntegratorRange(-rotationConstants.k_iZone, rotationConstants.k_iZone);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Constructs a HolonomicDriveController
   *
   * @param translationConstants PID constants for the translation PID controllers
   * @param rotationConstants    PID constants for the rotation controller
   */
  public RARHolonomicDriveController(
      PIDConstants translationConstants, ProfiledPIDConstants rotationConstants) {
    this(translationConstants, rotationConstants, 0.02);
  }

  /**
   * Enables and disables the controller for troubleshooting. When calculate() is
   * called on a
   * disabled controller, no values are returned.
   *
   * @param enabled If the controller is enabled or not
   */
  public void setEnabled(boolean enabled) {
    this.isEnabled = enabled;
  }

  /**
   * Resets the controller based on the current state of the robot
   *
   * @param currentPose   Current robot pose
   * @param currentSpeeds Current robot relative chassis speeds
   */
  public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    xController.reset();
    yController.reset();
    rotationController.reset(new State(currentPose.getRotation().getRadians(),
        currentSpeeds.omegaRadiansPerSecond));
  }

  /**
   * Calculates the next output of the path following controller
   *
   * @param currentPose The current robot pose
   * @param targetState The desired trajectory state
   * @param goalPose The pose to end at
   * @param maxApproachSpeed the speed in m/s to run at
   * @return The next robot relative output of the path following controller
   */
  public ChassisSpeeds calculateRobotRelativeSpeeds(
      Pose2d currentPose, Pose2d goalPose, double maxApproachSpeed) {

    Logger.recordOutput("Odometry/GoalPose", goalPose);

    if (!this.isEnabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, currentPose.getRotation());
    }

    Rotation2d targetHeading = goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    double translationError = currentPose.getTranslation().getDistance(goalPose.getTranslation()); // distance from goal pose

    // As we get closer to the target, we should be ramping down our x/y FF
    double targetSpeed = maxApproachSpeed;
    double falloffDistance = RobotConstants.robotConfig.AutoAlign.k_fallOffDistance;
    
    if (translationError < falloffDistance) {
      targetSpeed = (maxApproachSpeed / falloffDistance) * translationError;
    }
    
    Logger.recordOutput("AutoAligner/DistanceFromGoalPose", translationError);
    Logger.recordOutput("AutoAligner/TargetSpeed", targetSpeed);

    double xFF = targetSpeed * targetHeading.getCos();
    double yFF = targetSpeed * targetHeading.getSin();

    double xFeedback = this.xController.calculate(currentPose.getX(), goalPose.getX());
    double yFeedback = this.yController.calculate(currentPose.getY(), goalPose.getY());

    double rotationFeedback = rotationController.calculate(
        currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians());

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, rotationFeedback, currentPose.getRotation());
  }
}
