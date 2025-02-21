package frc.robot.wrappers;

public class PIDConstants {
  public final double k_P;
  public final double k_I;
  public final double k_D;
  public final double k_iZone;
  
  public PIDConstants(double kP, double kI, double kD, double kIZone) {
    k_P = kP;
    k_I = kI;
    k_D = kD;
    k_iZone = kIZone;
  }

  public PIDConstants(double kP, double kI, double kD) {
    k_P = kP;
    k_I = kI;
    k_D = kD;
    k_iZone = 0.0;
  }
}
