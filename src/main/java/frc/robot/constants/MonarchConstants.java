package frc.robot.constants;

public class MonarchConstants extends Constants {
  public MonarchConstants() {
    SwerveDrive.Drive.k_P = 0.84992; // setCory("goated");
    SwerveDrive.Drive.k_I = 0.0;
    SwerveDrive.Drive.k_D = 0.0;
    SwerveDrive.Drive.k_IZone = 0.0;

    SwerveDrive.Drive.k_FFS = 0.2368;
    SwerveDrive.Drive.k_FFV = 0.67229;
    SwerveDrive.Drive.k_FFA = 0.080151;

    // SwerveDrive.Turn.k_FLOffset = 0.282958984375;
    // SwerveDrive.Turn.k_FROffset = -0.1513671875;
    // SwerveDrive.Turn.k_BLOffset = -0.318115234375;
    // SwerveDrive.Turn.k_BROffset = -0.036865234375;

    SwerveDrive.Turn.k_FLOffset = -0.2783;
    SwerveDrive.Turn.k_FROffset = 0.154;
    SwerveDrive.Turn.k_BLOffset = 0.3174;
    SwerveDrive.Turn.k_BROffset = 0.0408;

    SwerveDrive.Turn.k_P = 70.0;
    SwerveDrive.Turn.k_I = 0.0;
    SwerveDrive.Turn.k_D = 1.0;
    SwerveDrive.Turn.k_IZone = 0.0;

    Arm.k_stowAngle = 0.26152366399765015;
    Arm.k_L4Angle = 0.5249119997024536;
    // TODO: This is a possible new L4 scoring pose
    // Arm.k_L4Angle = 0.5834578275680542;
    Arm.k_horizontalAngle = 0.5074164867401123;
    Arm.k_sourceAngle = 0.35026;

    Intake.Left.k_stowPosition = 0.551323413848877;
    Intake.Left.k_groundPosition = 0.18343025035858154;
    Intake.Left.k_ejectPosition = 0.31802046298980713;
    Intake.Left.k_horizontalPosition = 0.31802046298980713;
    Intake.Left.k_stuckPosition = 0.5542285442352295;
    Intake.Left.k_algaePosition = 0.4322316646575928;

    Intake.Right.k_stowPosition = 0.4848584532737732;
    Intake.Right.k_groundPosition = 0.854831151008606;
    Intake.Right.k_ejectPosition = 0.7199869155883789;
    Intake.Right.k_horizontalPosition = 0.7199869155883789;
    Intake.Right.k_stuckPosition = 0.7980979681015015;
  }
}
