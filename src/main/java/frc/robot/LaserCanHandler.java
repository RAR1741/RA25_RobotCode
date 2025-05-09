package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.RobotConstants;

public class LaserCanHandler {
  private static LaserCanHandler m_instance;

  private LaserCan m_entranceLaser;
  private LaserCan m_exitLaser;
  private LaserCan m_middleLaser;

  private int entranceDebounceCounter = 0;
  private int middleDebounceCounter = 0;
  private int exitDebounceCounter = 0;
  private int debounceMax = 3;

  public static LaserCanHandler getInstance() {
    if (m_instance == null) {
      m_instance = new LaserCanHandler();
    }
    return m_instance;
  }

  private LaserCanHandler() {
    m_entranceLaser = new LaserCan(RobotConstants.robotConfig.LaserCan.k_entranceId);
    m_middleLaser = new LaserCan(RobotConstants.robotConfig.LaserCan.k_middleId);
    m_exitLaser = new LaserCan(RobotConstants.robotConfig.LaserCan.k_exitId);

    if (RobotBase.isReal()) {
      try {
        m_entranceLaser.setRangingMode(LaserCan.RangingMode.SHORT);
        m_entranceLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        m_entranceLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);

        m_middleLaser.setRangingMode(LaserCan.RangingMode.SHORT);
        m_middleLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 2, 2));
        m_middleLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);

        m_exitLaser.setRangingMode(LaserCan.RangingMode.SHORT);
        m_exitLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 2, 2));
        m_exitLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);

      } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed! " + e);
      }
    }
  }

  @AutoLogOutput(key = "LaserCans/Entrance/distance")
  public double getEntranceDistance() {
    if (RobotBase.isSimulation()) {
      return -1;
    }
    return m_entranceLaser.getMeasurement().distance_mm;
  }

  @AutoLogOutput(key = "LaserCans/Exit/distance")
  public double getExitDistance() {
    if (RobotBase.isSimulation()) {
      return -1;
    }
    return m_exitLaser.getMeasurement().distance_mm;
  }

  @AutoLogOutput(key = "LaserCans/Middle/distance")
  public double getMiddleDistance() {
    if (RobotBase.isSimulation()) {
      return -1;
    }
    return m_middleLaser.getMeasurement().distance_mm;
  }

  @AutoLogOutput(key = "LaserCans/Entrance/seesCoral")
  public boolean getEntranceSeesCoral() {
    if (RobotBase.isSimulation()) {
      return true;
    }

    if (getEntranceDistance() < RobotConstants.robotConfig.LaserCan.k_entranceThreshold) {
      entranceDebounceCounter++;
    } else {
      entranceDebounceCounter = 0;
    }

    return entranceDebounceCounter >= debounceMax;
  }

  @AutoLogOutput(key = "LaserCans/Exit/seesCoral")
  public boolean getExitSeesCoral() {
    if (RobotBase.isSimulation()) {
      return true;
    }

    if (getExitDistance() < RobotConstants.robotConfig.LaserCan.k_exitThreshold) {
      exitDebounceCounter++;
    } else {
      exitDebounceCounter = 0;
    }

    return exitDebounceCounter >= debounceMax;
  }

  @AutoLogOutput(key = "LaserCans/Middle/seesCoral")
  public boolean getMiddleSeesCoral() {
    if (RobotBase.isSimulation()) {
      return true;
    }

    if (getMiddleDistance() < RobotConstants.robotConfig.LaserCan.k_middleThreshold) {
      middleDebounceCounter++;
    } else {
      middleDebounceCounter = 0;
    }

    return middleDebounceCounter >= debounceMax;
  }
}
