package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;

import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.LaserCanHandler;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.wrappers.RARSparkMax;

public class EndAffector extends Subsystem {
    private static EndAffector m_instance;
    private PeriodicIO m_periodicIO;

    RARSparkMax m_leftMotor;
    RARSparkMax m_rightMotor;

    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    private SlewRateLimiter m_leftSpeedLimiter = new SlewRateLimiter(1000);
    private SlewRateLimiter m_rightSpeedLimiter = new SlewRateLimiter(1000);

    private LaserCanHandler m_laserCan;
    private Elevator m_elevator;

    public static EndAffector getInstance() {
        if (m_instance == null) {
            m_instance = new EndAffector();
        }
        return m_instance;
    }

    private EndAffector() {
        super("EndEffector");

        m_periodicIO = new PeriodicIO();

        m_leftMotor = new RARSparkMax(RobotConstants.robotConfig.EndAffector.k_leftMotorId, MotorType.kBrushless);
        m_rightMotor = new RARSparkMax(RobotConstants.robotConfig.EndAffector.k_rightMotorId, MotorType.kBrushless);

        m_laserCan = LaserCanHandler.getInstance();
        m_elevator = Elevator.getInstance();

        SparkBaseConfig shooterConfig = new SparkFlexConfig().idleMode(IdleMode.kCoast);

        shooterConfig.closedLoop
                .pidf(RobotConstants.robotConfig.EndAffector.k_P,
                        RobotConstants.robotConfig.EndAffector.k_I,
                        RobotConstants.robotConfig.EndAffector.k_D,
                        RobotConstants.robotConfig.EndAffector.k_FF)
                .minOutput(RobotConstants.robotConfig.EndAffector.k_minOutput)
                .maxOutput(RobotConstants.robotConfig.EndAffector.k_maxOutput);

        SparkBaseConfig leftShooterConfig = new SparkFlexConfig()
                .apply(shooterConfig)
                .inverted(true);

        m_leftMotor.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leftEncoder = m_leftMotor.getEncoder();
        m_rightEncoder = m_rightMotor.getEncoder();
    }

    private static class PeriodicIO {
        double leftSpeed = 0.0;
        double rightSpeed = 0.0;

        EndEffectorState state = EndEffectorState.OFF;
    }

    public enum EndEffectorState {
        OFF,
        INDEX,
        SCORE_BRANCHES,
        SCORE_TROUGH
    }

    public void setState(EndEffectorState state) {
        if (!m_laserCan.getIndexSeesCoral()) {
            m_periodicIO.state = state;
        }
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    @Override
    public void periodic() {
        updateState();

        if (m_periodicIO.state == EndEffectorState.OFF) {
            m_periodicIO.leftSpeed = 0.0;
            m_periodicIO.rightSpeed = 0.0;
        } else {
            if (m_elevator.getElevatorState() == ElevatorState.STOW) {
                if (m_elevator.getIsAtState()) {
                    if (m_laserCan.getIndexSeesCoral() || m_laserCan.getEntranceSeesCoral()) {
                        setState(EndEffectorState.INDEX);
                    }
                }
            } else {
                if (m_laserCan.getExitSeesCoral()) {
                    if (m_periodicIO.state == EndEffectorState.SCORE_BRANCHES) {
                        m_periodicIO.leftSpeed = RobotConstants.robotConfig.EndAffector.k_branchesSpeed;
                        m_periodicIO.rightSpeed = RobotConstants.robotConfig.EndAffector.k_branchesSpeed;
                    } else if (m_periodicIO.state == EndEffectorState.SCORE_TROUGH) {
                        m_periodicIO.leftSpeed = RobotConstants.robotConfig.EndAffector.k_troughSpeed;
                        m_periodicIO.rightSpeed = RobotConstants.robotConfig.EndAffector.k_branchesSpeed;
                    }
                } else {
                    setState(EndEffectorState.OFF);
                }
            }
        }
    }

    private void updateState() {
        if (m_laserCan.getEntranceSeesCoral()) {
            setState(EndEffectorState.INDEX);
        } else if (m_periodicIO.state == EndEffectorState.INDEX) {
            setState(EndEffectorState.OFF);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        double limitedRightSpeed = m_rightSpeedLimiter.calculate(m_periodicIO.rightSpeed);
        double limitedLeftSpeed = m_leftSpeedLimiter.calculate(m_periodicIO.leftSpeed);
        m_leftMotor.getClosedLoopController().setReference(limitedLeftSpeed, ControlType.kVelocity);
        m_rightMotor.getClosedLoopController().setReference(limitedRightSpeed, ControlType.kVelocity);
    }

    @AutoLogOutput(key = "EndEffector/State")
    public EndEffectorState getEndEffectorState() {
        return m_periodicIO.state;
    }

    @AutoLogOutput(key = "EndEffector/LeftMotorSpeed")
    public double getEndEffectorLeftMotorSpeed() {
        return m_leftEncoder.getVelocity();
    }

    @AutoLogOutput(key = "EndEffector/RightMotorSpeed")
    public double getEndEffectorRightMotorSpeed() {
        return m_rightEncoder.getVelocity();
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

}
