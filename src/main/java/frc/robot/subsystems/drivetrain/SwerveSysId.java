package frc.robot.subsystems.drivetrain;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Subsystem;

public class SwerveSysId extends Subsystem {
  private SwerveModule[] m_modules;

  public SwerveSysId(SwerveModule[] modules, String baseSmartDashboardKey) {
    super(baseSmartDashboardKey);

    m_modules = modules;
  }

  public Command sysIdTurnQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdTurnRoutine.quasistatic(direction);
  }

  public Command sysIdTurnDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdTurnRoutine.dynamic(direction);
  }

  public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdDriveRoutine.quasistatic(direction);
  }

  public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdDriveRoutine.dynamic(direction);
  }

  // Mutable holder for unit-safe SysID values (to avoid reallocation)
  private final MutVoltage m_appliedVoltage = Volts.of(0).mutableCopy();
  private final MutDistance m_distance = Meters.of(0).mutableCopy();
  private final MutLinearVelocity m_velocity = MetersPerSecond.of(0).mutableCopy();

  private final SysIdRoutine m_sysIdDriveRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (Voltage volts) -> {
            for (SwerveModule module : m_modules) {
              module.sysidDrive(volts.in(Volts));
            }
          },
          log -> {
            log.motor("drive-frontleft").voltage(
                m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.FRONT_LEFT].getDriveMotorVoltage()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(
                        m_modules[SwerveDrive.Module.FRONT_LEFT].getDrivePositionMet(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(
                        m_modules[SwerveDrive.Module.FRONT_LEFT].getDriveVelocity(), MetersPerSecond));

            log.motor("drive-frontright").voltage(
                m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.FRONT_RIGHT].getDriveMotorVoltage()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(
                        m_modules[SwerveDrive.Module.FRONT_RIGHT].getDrivePositionMet(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(
                        m_modules[SwerveDrive.Module.FRONT_RIGHT].getDriveVelocity(), MetersPerSecond));

            log.motor("drive-backright").voltage(
                m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.BACK_RIGHT].getDriveMotorVoltage()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(
                        m_modules[SwerveDrive.Module.BACK_RIGHT].getDrivePositionMet(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(
                        m_modules[SwerveDrive.Module.BACK_RIGHT].getDriveVelocity(), MetersPerSecond));

            log.motor("drive-backleft").voltage(
                m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.BACK_LEFT].getDriveMotorVoltage()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(
                        m_modules[SwerveDrive.Module.BACK_LEFT].getDrivePositionMet(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(
                        m_modules[SwerveDrive.Module.BACK_LEFT].getDriveVelocity(), MetersPerSecond));
          },
          this));

  private final SysIdRoutine m_sysIdTurnRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (Voltage volts) -> {
            for (SwerveModule module : m_modules) {
              module.sysidTurn(volts.in(Volts));
            }
          },
          log -> {
            log.motor("turn-frontleft").voltage(
                m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.FRONT_LEFT].getTurnMotorVoltage()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(
                        m_modules[SwerveDrive.Module.FRONT_LEFT].getTurnPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(
                        m_modules[SwerveDrive.Module.FRONT_LEFT].getTurnVelocity(), MetersPerSecond));

            log.motor("turn-frontright").voltage(
                m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.FRONT_RIGHT].getTurnMotorVoltage()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(
                        m_modules[SwerveDrive.Module.FRONT_RIGHT].getTurnPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(
                        m_modules[SwerveDrive.Module.FRONT_RIGHT].getTurnVelocity(), MetersPerSecond));

            log.motor("turn-backright").voltage(
                m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.BACK_RIGHT].getTurnMotorVoltage()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(
                        m_modules[SwerveDrive.Module.BACK_RIGHT].getTurnPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(
                        m_modules[SwerveDrive.Module.BACK_RIGHT].getTurnVelocity(), MetersPerSecond));

            log.motor("turn-backleft").voltage(
                m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.BACK_LEFT].getTurnMotorVoltage()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(
                        m_modules[SwerveDrive.Module.BACK_LEFT].getTurnPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(
                        m_modules[SwerveDrive.Module.BACK_LEFT].getTurnVelocity(), MetersPerSecond));
          },
          this));

  @Override
  public void reset() {
  };

  @Override
  public void periodic() {
  };

  @Override
  public void writePeriodicOutputs() {
  };

  @Override
  public void stop() {
  };
}