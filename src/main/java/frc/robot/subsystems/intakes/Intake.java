package frc.robot.subsystems.intakes;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotConstants;
import frc.robot.wrappers.RARSparkMax;
import frc.robot.wrappers.REVThroughBoreEncoder;

//* dear all programmers seeing this, this is my first year as a SPEC member, expect this file of java code to be a dumpster fire
//* i am sorry
//* - Sarthak Ghoshal & Chase Cleveland (2025)

public class Intake {
  private final String m_intakeName;
  
  private final RARSparkMax m_pivotMotor;
  private final RARSparkMax m_intakeMotor;
  private final REVThroughBoreEncoder m_pivotAbsEncoder;
  private final PeriodicIO m_periodicIO;

  private final ProfiledPIDController m_pivotMotorPID;
  private final ArmFeedforward m_pivotFeedForward;


  private static class PeriodicIO {
    IntakeState desiredIntakeState = IntakeState.NONE;
    IntakePivotTarget desiredPivotTarget = IntakePivotTarget.NONE;

    double pivotVoltage = 0.0;
    double intakeSpeed = 0.0;
  }

  public Intake(String intakeName, int pivotMotorID, int intakeMotorID, int absoluteEncoderID) {
    m_intakeName = intakeName;

    m_pivotMotor = new RARSparkMax(pivotMotorID, MotorType.kBrushless);
    m_intakeMotor = new RARSparkMax(intakeMotorID, MotorType.kBrushless);

    m_pivotAbsEncoder = new REVThroughBoreEncoder(absoluteEncoderID);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    SparkMaxConfig intakeConfig = new SparkMaxConfig();

    pivotConfig.idleMode(IdleMode.kCoast);
    pivotConfig.inverted(true);
    pivotConfig.encoder.positionConversionFactor(RobotConstants.robotConfig.SwerveDrive.k_turnGearRatio * 2.0 * Math.PI);
    pivotConfig.encoder.velocityConversionFactor(RobotConstants.robotConfig.SwerveDrive.k_turnGearRatio * 2.0 * Math.PI / 60.0);
    m_pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeConfig.idleMode(IdleMode.kCoast);
    intakeConfig.inverted(true);
    intakeConfig.encoder.positionConversionFactor(RobotConstants.robotConfig.SwerveDrive.k_turnGearRatio * 2.0 * Math.PI);
    intakeConfig.encoder.velocityConversionFactor(RobotConstants.robotConfig.SwerveDrive.k_turnGearRatio * 2.0 * Math.PI / 60.0);
    m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    // Pivot PID
    m_pivotMotorPID = new ProfiledPIDController(
        RobotConstants.robotConfig.Intake.k_pivotMotorP,
        RobotConstants.robotConfig.Intake.k_pivotMotorI,
        RobotConstants.robotConfig.Intake.k_pivotMotorD,
        new TrapezoidProfile.Constraints(
            RobotConstants.robotConfig.Intake.k_maxVelocity,
            RobotConstants.robotConfig.Intake.k_maxAcceleration));

    // Pivot Feedforward
    m_pivotFeedForward = new ArmFeedforward(
        RobotConstants.robotConfig.Intake.k_pivotMotorKS,
        RobotConstants.robotConfig.Intake.k_pivotMotorKG,
        RobotConstants.robotConfig.Intake.k_pivotMotorKV,
        RobotConstants.robotConfig.Intake.k_pivotMotorKA);

    m_periodicIO = new PeriodicIO();
    //TODO: Wait until u get logic in to add pivot encoder :p
  }

  public void setPivotTarget(IntakePivotTarget target) {
    m_periodicIO.desiredPivotTarget = target;
  }

  public void setIntakeState(IntakeState target) {
    m_periodicIO.desiredIntakeState = target;
  }

  public void periodic() {
    double pidCalc = m_pivotMotorPID.calculate(getPivotAngle(), getTargetPivotAngle());
    double ffCalc = m_pivotFeedForward.calculate(Math.toRadians(getPivotReferenceToHorizontal()),
        Math.toRadians(m_pivotMotorPID.getSetpoint().velocity));

    m_periodicIO.pivotVoltage = pidCalc + ffCalc;

    m_periodicIO.intakeSpeed = getDesiredIntakeSpeed();
  }

  public void writePeriodicOutputs() {
    m_pivotMotor.setVoltage(m_periodicIO.pivotVoltage);
    m_intakeMotor.set(m_periodicIO.intakeSpeed);
  }

  public void stop() {
    m_periodicIO.pivotVoltage = 0.0;
    m_periodicIO.desiredIntakeState = IntakeState.NONE;
    m_periodicIO.desiredPivotTarget = IntakePivotTarget.NONE;
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/DesiredIntakeState")
  public String getDesiredIntakeState() {
    return m_periodicIO.desiredIntakeState.toString();
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/DesiredPivotTarget")
  public String getDesiredPivotTarget() {
    return m_periodicIO.desiredPivotTarget.toString();
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/DesiredIntakeSpeed")
  public double getDesiredIntakeSpeed() {
    switch(m_periodicIO.desiredIntakeState) {
      case NONE -> {
        return 0.0;
      }
      case INTAKE -> {
        return 0.8;
      }
      default -> {
        return 0.0;
      }
    }
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/getTargetPivotAngle")
  public double getTargetPivotAngle() {
    switch(m_periodicIO.desiredPivotTarget) {
      case NONE -> {
        return 0.0;
      }
      case GROUND -> {
        return RobotConstants.robotConfig.Intake.k_groundAngle;
      }
      default -> {
        return 0.0;
      }
    }
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/getPivotAngle")
  public double getPivotAngle() {
    return Units.rotationsToDegrees(m_pivotAbsEncoder.get()); //TODO: was getAbsolutePosition() so make sure this works
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/getPivotReferenceToHorizal")
  public double getPivotReferenceToHorizontal() {
    return getPivotAngle() - RobotConstants.robotConfig.Intake.k_pivotOffset;
  }

  public enum IntakePivotTarget {
    NONE,
    GROUND,
    EJECT,
    PIVOT,
    AMP,
    STOW
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT,
    PULSE,
    FEED_SHOOTER
  }
}
