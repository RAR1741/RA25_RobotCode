package frc.robot.wrappers;

public class ProfiledPIDConstants {
  public final double k_P;
  public final double k_I;
  public final double k_D;
  public final double k_iZone;
  /** Max Vel */
  public final double k_maxVel;
  /** Max Acc */
  public final double k_maxAcc;

  /**
   * Create a new ProfiledPIDConstants object
   *
   * @param kP P
   * @param kI I
   * @param kD D
   * @param iZone Integral range
   * @param maxVel Maximum Velocity
   * @param maxAcc Maximum Acceleration
   */
  public ProfiledPIDConstants(
    double kP,
    double kI,
    double kD,
    double iZone,
    double maxVel,
    double maxAcc) {
    k_P = kP;
    k_I = kI;
    k_D = kD;
    k_iZone = iZone;
    k_maxVel = maxVel;
    k_maxAcc = maxAcc;
  }

  /**
   * Create a new ProfiledPIDConstants object
   *
   * @param kP P
   * @param kI I
   * @param kD D
   * @param maxVel Maximum Velocity
   * @param maxAcc Maximum Acceleration
   */
  public ProfiledPIDConstants(
    double kP,
    double kI,
    double kD,
    double maxVel,
    double maxAcc) {
      k_P = kP;
      k_I = kI;
      k_D = kD;
      k_iZone = 0.0;
      k_maxVel = maxVel;
      k_maxAcc = maxAcc;
  }
}
