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

    Arm.k_stowAngle = 0.2897946238517761;
    Arm.k_L4Angle = 0.5404757261276245;
    Arm.k_horizontalAngle = 0.5404757261276245;

    Intake.Left.k_stowPosition = 0.585437536239624;
    Intake.Left.k_groundPosition = 0.22542721033096313;
    Intake.Left.k_ejectPosition = 0.36690253019332886;
    Intake.Left.k_horizontalPosition = 0.36690253019332886;

    Intake.Right.k_stowPosition = 0.48890572786331177;
    Intake.Right.k_groundPosition = 0.855; //0.8605759143829346;
    Intake.Right.k_ejectPosition = 0.710295557975769;
    Intake.Right.k_horizontalPosition = 0.710295557975769;
  }
}
