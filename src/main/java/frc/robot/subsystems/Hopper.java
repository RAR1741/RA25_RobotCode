package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.RobotConstants;


//add PID

public class Hopper extends Subsystem {
  private static Hopper m_instance = null;
  private final SparkMax m_hopperMotor;
  private final SparkClosedLoopController m_hopperController;
  private static PeriodicIO m_periodicIO;

  public static class PeriodicIO {
    boolean is_hopper_on = false;
  }

  private Hopper() {
    super("Hopper");

    //create the motor
    m_hopperMotor = new SparkMax(RobotConstants.robotConfig.Hopper.k_hopperMotorId, MotorType.kBrushless);
    //get the closed loop controller from the motor
    m_hopperController = m_hopperMotor.getClosedLoopController();
    //create config
    SparkMaxConfig config = new SparkMaxConfig();
    
    //add config
    config.inverted(true); //TODO: check this
    m_hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


   //set PID values
   config.closedLoop
        .pidf(RobotConstants.robotConfig.Hopper.k_P,
            RobotConstants.robotConfig.Hopper.k_I,
            RobotConstants.robotConfig.Hopper.k_D,
            RobotConstants.robotConfig.Hopper.k_FF)
        .iZone(RobotConstants.robotConfig.Hopper.k_IZone);

    m_periodicIO = new PeriodicIO();
  }
  
  public static Hopper getInstance() {
    if(m_instance == null) {
      m_instance = new Hopper();
    }
    return m_instance;
  }

  public void start() {
    m_periodicIO.is_hopper_on = true;
  }

  @Override
  public void reset() {
    m_periodicIO.is_hopper_on = false;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
    if(m_periodicIO.is_hopper_on) {
        m_hopperController.setReference(RobotConstants.robotConfig.Hopper.k_maxHopperSpeedRPM, ControlType.kVelocity);
    } else {
        m_hopperController.setReference(0.0, ControlType.kVoltage);
    }
  }

  @Override
  public void stop() {
    m_periodicIO.is_hopper_on = false;
  }

  @AutoLogOutput
  public double getMotorVelocityRPM() {
    return m_hopperMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput
  public double getMotorVelSetPoint() {
    if(m_periodicIO.is_hopper_on) { 
        return m_hopperMotor.k_setVelocity; 
    } else {
        return m_hopperMotor.k_setVelocity;
    }
  }
}