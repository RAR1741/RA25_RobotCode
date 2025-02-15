package frc.robot.constants;

public class JormungandrConstants extends Constants {
  public JormungandrConstants() {
    SwerveDrive.Drive.k_P = 0.84992; // setCory("goated");
    SwerveDrive.Drive.k_I = 0.0;
    SwerveDrive.Drive.k_D = 0.0;
    SwerveDrive.Drive.k_IZone = 0.0;

    SwerveDrive.Drive.k_FFS = 0.2368;
    SwerveDrive.Drive.k_FFV = 0.67229;
    SwerveDrive.Drive.k_FFA = 0.080151;

    SwerveDrive.Turn.k_FLOffset = -0.1521;
    SwerveDrive.Turn.k_FROffset = 0.02417;
    SwerveDrive.Turn.k_BLOffset = 0.594889;
    SwerveDrive.Turn.k_BROffset = 0.419678;
    
    SwerveDrive.Turn.k_P = 70.0;
    SwerveDrive.Turn.k_I = 0.0;
    SwerveDrive.Turn.k_D = 1.0;
    SwerveDrive.Turn.k_IZone = 0.0;

    Elevator.k_P = 0.15;
    Elevator.k_I = 0.0;
    Elevator.k_D = 0.0;
    Elevator.k_IZone = 0.0;
    Elevator.k_FF = 0.50;

    Arm.k_P = 0.0; //0.03
    Arm.k_I = 0.0;
    Arm.k_D = 0.0;
    Arm.k_IZone = 0.0;
    Arm.k_FF = 0.025; //0.25
    
    Intake.k_pivotMotorP = 0.0;
    Intake.k_pivotMotorI = 0.0;
    Intake.k_pivotMotorD = 0.0;
    Intake.k_pivotMotorFF = 0.0;

    Intake.k_leftPivotOffset = (360.0 - 330.529052734375) / 360.0;
    Intake.k_rightPivotOffset = (360.0 - 178.09091186523438) / 360.0;
  }
}
