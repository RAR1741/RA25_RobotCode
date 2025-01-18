package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.RobotConstants;
import frc.robot.wrappers.RARSparkMax;

public class Conveyor extends Subsystem {
  private static Conveyor m_instance = null;
  private final RARSparkMax m_conveyorMotor;
  private static PeriodicIO m_periodicIO;

  public static class PeriodicIO {
    double conveyorSpeed = 0.0;
  }

  private Conveyor() {
    super("Conveyor");

    m_conveyorMotor = new RARSparkMax(RobotConstants.robotConfig.Conveyor.k_conveyorMotorId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true); //TODO: check this
    m_conveyorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_periodicIO = new PeriodicIO();
  }
  
  public static Conveyor getInstance() {
    if(m_instance == null) {
      m_instance = new Conveyor();
    }
    return m_instance;
  }

  public void start() {
    m_periodicIO.conveyorSpeed = RobotConstants.robotConfig.Conveyor.k_conveyorSpeed;
  }

  @Override
  public void reset() {
  }

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
    m_conveyorMotor.set(m_periodicIO.conveyorSpeed);
  }

  @Override
  public void stop() {
    m_periodicIO.conveyorSpeed = 0.0;
    m_conveyorMotor.set(0.0);
  }

  @AutoLogOutput
  public double getConveyorSpeed() {
    return m_periodicIO.conveyorSpeed;
  }

  @AutoLogOutput
  public double getMotorVelocityRPM() {
    return m_conveyorMotor.getEncoder().getVelocity();
  }
}