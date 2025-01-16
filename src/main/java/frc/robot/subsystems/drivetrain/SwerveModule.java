package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Helpers;
import frc.robot.constants.RobotConstants;
import frc.robot.wrappers.RARSparkMax;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final RARSparkMax m_turnMotor;
  
  private final SparkClosedLoopController m_turningPIDController;
  private final SparkAbsoluteEncoder m_turningAbsEncoder;
  private final RelativeEncoder m_turningRelEncoder;

  private final PeriodicIO m_periodicIO = new PeriodicIO();

  private final double m_turningOffset;
  
  @SuppressWarnings("unused")
  private final String m_moduleName;

  private static class PeriodicIO {
    SwerveModuleState desiredState = new SwerveModuleState();
    // boolean shouldChangeState = false; // TODO: maybe add this back?
  }

  private boolean m_moduleDisabled = false;

  public SwerveModule(String moduleName, int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, double turningOffset) {
    m_moduleName = moduleName;
    m_turningOffset = turningOffset;

    m_driveMotor = new TalonFX(driveMotorChannel);
    m_driveMotor.setNeutralMode(NeutralModeValue.Coast);
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    driveConfig.Feedback.SensorToMechanismRatio = RobotConstants.robotConfig.SwerveDrive.k_driveGearRatio;
    // driveConfig.Feedback.RotorToSensorRatio = 0.0f; // TODO: check back with this if we add CANcoders

    driveConfig.Slot0.kP = RobotConstants.robotConfig.SwerveDrive.Drive.k_P;
    driveConfig.Slot0.kI = RobotConstants.robotConfig.SwerveDrive.Drive.k_I;
    driveConfig.Slot0.kD = RobotConstants.robotConfig.SwerveDrive.Drive.k_D;

    driveConfig.Slot0.kS = RobotConstants.robotConfig.SwerveDrive.Drive.k_FFS;
    driveConfig.Slot0.kV = RobotConstants.robotConfig.SwerveDrive.Drive.k_FFV;
    driveConfig.Slot0.kA = RobotConstants.robotConfig.SwerveDrive.Drive.k_FFA;

    m_driveMotor.getConfigurator().apply(driveConfig);

    m_turnMotor = new RARSparkMax(turningMotorChannel, MotorType.kBrushless);
    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.idleMode(IdleMode.kCoast);
    turnConfig.inverted(true);

    turnConfig.encoder.positionConversionFactor(RobotConstants.robotConfig.SwerveDrive.k_turnGearRatio * 2.0 * Math.PI);
    turnConfig.encoder.velocityConversionFactor(RobotConstants.robotConfig.SwerveDrive.k_turnGearRatio * 2.0 * Math.PI / 60.0);

    turnConfig.closedLoop.p(RobotConstants.robotConfig.SwerveDrive.Turn.k_P);
    turnConfig.closedLoop.i(RobotConstants.robotConfig.SwerveDrive.Turn.k_I);
    turnConfig.closedLoop.d(RobotConstants.robotConfig.SwerveDrive.Turn.k_D);
    turnConfig.closedLoop.positionWrappingEnabled(true);
    turnConfig.closedLoop.positionWrappingMinInput(0.0);
    turnConfig.closedLoop.positionWrappingMaxInput(2.0 * Math.PI);
    turnConfig.closedLoop.outputRange(
      RobotConstants.robotConfig.SwerveDrive.Turn.k_minOutput,
      RobotConstants.robotConfig.SwerveDrive.Turn.k_maxOutput);
    
    // m_turnMotor.setSmartCurrentLimit(RobotConstants.SwerveDrive.Drive.k_turnCurrentLimit);
    // m_turningAbsEncoder = new TalonSRXMagEncoder(turningEncoderChannel);

    m_turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_turningPIDController = m_turnMotor.getClosedLoopController();
    m_turningAbsEncoder = m_turnMotor.getAbsoluteEncoder();
    m_turningRelEncoder = m_turnMotor.getEncoder();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(), Rotation2d.fromRadians(getTurnPosition()));
  }

  public SwerveModulePosition getPosition() {
    double drivePosition = m_driveMotor.getPosition().getValueAsDouble();

    return new SwerveModulePosition(
        drivePosition, Rotation2d.fromRadians(getTurnPosition()));
  }

  public TalonFX getDriveMotor() {
    return m_driveMotor;
  }

  public RARSparkMax getTurnMotor() {
    return m_turnMotor;
  }

  public void clearTurnPIDAccumulation() {
    m_turningPIDController.setIAccum(0);
  }

  public void resetDriveEncoder() {
    m_driveMotor.setPosition(0.0);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(Rotation2d.fromRadians(getTurnPosition()));
    
    desiredState.angle = new Rotation2d(Helpers.modRadians(desiredState.angle.getRadians()));
    // m_periodicIO.shouldChangeState = !desiredState.equals(m_periodicIO.desiredState);
    
    m_periodicIO.desiredState = desiredState;
  }

  public SwerveModuleState getDesiredState() {
    return m_periodicIO.desiredState;
  }

  public void periodic() {
    double velocity = Helpers.MPSToRPS(getDriveTargetVelocity(), RobotConstants.robotConfig.SwerveDrive.k_wheelCircumference);
    // double angularVelocity = getDriveTargetVelocity() / Units.inchesToMeters(RobotConstants.robotConfig.SwerveDrive.k_wheelRadiusIn);

    VelocityVoltage request = new VelocityVoltage(velocity).withSlot(0);
    m_driveMotor.setControl(request);
    m_turningPIDController.setReference(getTurnTargetAngleRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  // Logged
  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/targetAngle")
  private double getTurnTargetAngleRadians() {
    return m_periodicIO.desiredState.angle.getRadians();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/speedMPS")
  private double getDriveTargetVelocity() {
    return m_periodicIO.desiredState.speedMetersPerSecond;
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Voltage")
  public double getTurnMotorVoltage() {
    return Helpers.getVoltage(m_turnMotor);
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Current")
  public double getTurnMotorCurrent() {
    return m_turnMotor.getOutputCurrent();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Voltage")
  public double getDriveMotorVoltage() {
    return m_driveMotor.getMotorVoltage().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Current")
  public double getDriveMotorCurrent() {
    return m_driveMotor.getTorqueCurrent().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Abs/getTurnPosition")
  public double getAsbEncoderPosition() {
    return m_turningAbsEncoder.getPosition() - m_turningOffset;
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Temperature")
  public double getDriveTemp() {
    return m_driveMotor.getDeviceTemp().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Temperature")
  public double getTurnTemp() {
    return m_turnMotor.getMotorTemperature();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Velocity")
  public double getDriveVelocity() {
    return Helpers.RPSToMPS(m_driveMotor.getVelocity().getValueAsDouble(), RobotConstants.robotConfig.SwerveDrive.k_wheelCircumference);
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/PositionRot")
  public double getDrivePositionRot() {
    return m_driveMotor.getPosition().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/PositionMet")
  public double getDrivePositionMet() {
    return Helpers.RPSToMPS(m_driveMotor.getPosition().getValueAsDouble(), RobotConstants.robotConfig.SwerveDrive.k_wheelCircumference);
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Position")
  public double getTurnPosition() {
    return Helpers.modRadians(m_turningRelEncoder.getPosition());
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/absPosition")
  public double getTurnAbsPosition() {
    return Helpers.modRotations(m_turningAbsEncoder.getPosition() - m_turningOffset);
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Velocity")
  public double getTurnVelocity() {
    return m_turningRelEncoder.getVelocity();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/error")
  public double getTurnError() {
    return getState().angle.minus(m_periodicIO.desiredState.angle).getDegrees();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/errorRelToAbs")
  public double getTurnErrorRelToAbs() {
    return getState().angle.minus(Rotation2d.fromRotations(getTurnAbsPosition())).getDegrees();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/ModuleDisabled")
  public boolean isModuleDisabled() {
    return m_moduleDisabled;
  }
}
