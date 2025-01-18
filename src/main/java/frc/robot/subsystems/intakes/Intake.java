package frc.robot.subsystems.intakes;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.RobotConstants;
import frc.robot.wrappers.RARSparkMax;
import frc.robot.wrappers.REVThroughBoreEncoder;

//* dear all programmers seeing this, this is my first year as a SPEC member, expect this file of java code to be a dumpster fire
//* i am sorry
//* - Sarthak Ghoshal 2025

public class Intake {
  
  private final RARSparkMax m_pivotMotor;
  private final RARSparkMax m_intakeMotor;
  private final REVThroughBoreEncoder m_pivotEncoder;

  public Intake(int pivotMotorID, int intakeMotorID, int absoluteEncoderID) {
    m_pivotMotor = new RARSparkMax(pivotMotorID, MotorType.kBrushless);
    m_intakeMotor = new RARSparkMax(intakeMotorID, MotorType.kBrushless);

    m_pivotEncoder = new REVThroughBoreEncoder(absoluteEncoderID);

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
  
    //TODO: Wait until u get logic in to add pivot encoder :p
  }

  public enum IntakePivotTarget {
    NONE,
    GROUND,
    EJECT,
    PIVOT,
    AMP,
    STOW
  }

  public enum State {
    NONE,
    INTAKE,
    EJECT,
    PULSE,
    FEED_SHOOTER
  }

  public enum Motor {
    INTAKE,
    PIVOT
  }

  public void changeState(Motor intakeOrPivot, State state, boolean get) { //& haha react-redux coding style go brrrrrrr 
    //TODO: GET PERIODIO, THEN DO BS
  }
}
