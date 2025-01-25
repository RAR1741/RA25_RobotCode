package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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
   * @param rotationConstants PID constants for the rotation controller
   * @param period Period of the control loop in seconds
   */
  public RARHolonomicDriveController(
      PIDConstants translationConstants, ProfiledPIDConstants rotationConstants, double period) {
    this.xController =
        new PIDController(
            translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
    this.xController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);

    this.yController =
        new PIDController(
            translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
    this.yController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);

    // Temp rate limit of 0, will be changed in calculate
    this.rotationController =
        new ProfiledPIDController(
          rotationConstants.kP,
          rotationConstants.kI,
          rotationConstants.kD,
          new Constraints(rotationConstants.maxVel, rotationConstants.maxAcc),
          period);
    this.rotationController.setIntegratorRange(-rotationConstants.iZone, rotationConstants.iZone);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Constructs a HolonomicDriveController
   *
   * @param translationConstants PID constants for the translation PID controllers
   * @param rotationConstants PID constants for the rotation controller
   */
  public RARHolonomicDriveController(
      PIDConstants translationConstants, ProfiledPIDConstants rotationConstants) {
    this(translationConstants, rotationConstants, 0.02);
  }

  /**
   * Enables and disables the controller for troubleshooting. When calculate() is called on a
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
   * @param currentPose Current robot pose
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
   * @return The next robot relative output of the path following controller
   */
  public ChassisSpeeds calculateRobotRelativeSpeeds(
      Pose2d currentPose, Pose2d goalPose, double maxApproachSpeed) {

    if (!this.isEnabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, currentPose.getRotation());
    }

    Rotation2d targetHeading = goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    double translationError = currentPose.getTranslation().getDistance(goalPose.getTranslation());

    // As we get closer to the target, we should be ramping down our x/y FF
    double targetSpeed = Math.min(maxApproachSpeed, translationError * 2);

    double xFF = targetSpeed * targetHeading.getCos();
    double yFF = targetSpeed * targetHeading.getSin();

    double xFeedback = this.xController.calculate(currentPose.getX(), goalPose.getX());
    double yFeedback = this.yController.calculate(currentPose.getY(), goalPose.getY());

    double rotationFeedback =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians());

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, rotationFeedback, currentPose.getRotation());
  }
}
