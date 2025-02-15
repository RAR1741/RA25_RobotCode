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

    Arm.k_P = 10.8;
    Arm.k_I = 0.0;
    Arm.k_D = 0.0;
    Arm.k_IZone = 0.0;

    Arm.k_FFS = 0.0;
    Arm.k_FFV = 0.0;
    Arm.k_FFA = 0.0;
    Arm.k_FFG = 0.35;

    Arm.k_stowAngle = 0.29270070791244507;
    Arm.k_L4Angle = 0.5404757261276245;
    Arm.k_horizontalAngle = 0.5404757261276245;

    Intake.Left.k_stowPosition = 0.8342224359512329;
    Intake.Left.k_groundPosition = 0.20293329656124115;
    Intake.Left.k_ejectPosition = 0.050809308886528015;
    Intake.Left.k_horizontalPosition = 0.050809308886528015;

    Intake.Right.k_stowPosition = 0.9970211386680603;
    Intake.Right.k_groundPosition = 0.3804924488067627;
    Intake.Right.k_ejectPosition = 0.21850648522377014;
    Intake.Right.k_horizontalPosition = 0.21850648522377014;
  }
}
