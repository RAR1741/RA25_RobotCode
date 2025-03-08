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

  private int debounceCounter = 0;
  private int debounceMax = 3;

  public static LaserCanHandler getInstance() {
    if (m_instance == null) {
      m_instance = new LaserCanHandler();
    }
    return m_instance;
  }

  private LaserCanHandler() {
    m_entranceLaser = new LaserCan(RobotConstants.robotConfig.LaserCan.k_entranceId);
    // m_exitLaser = new LaserCan(RobotConstants.robotConfig.LaserCan.k_exitId);

    try {
      m_entranceLaser.setRangingMode(LaserCan.RangingMode.SHORT);
      m_entranceLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 2, 2));
      m_entranceLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);

      // m_exitLaser.setRangingMode(LaserCan.RangingMode.SHORT);
      // m_exitLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      // m_exitLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  @AutoLogOutput(key = "LaserCans/Entrance/seesCoral")
  public boolean getEntranceSeesCoral() {
    if (RobotBase.isSimulation()) {
      return true;
    }

    if (getEntranceDistance() < RobotConstants.robotConfig.LaserCan.k_entranceThreshold) {
      debounceCounter++;
    } else {
      debounceCounter = 0;
    }

    return debounceCounter >= debounceMax;
  }

  @AutoLogOutput(key = "LaserCans/Entrance/distance")
  public double getEntranceDistance() {
    return m_entranceLaser.getMeasurement().distance_mm;
  }

  // @AutoLogOutput(key = "LaserCans/Exit/seesCoral")
  // public boolean getExitSeesCoral() {
  // return m_exitLaser.getMeasurement().distance_mm < 75.0; // Value gotten from
  // Cranberry Alarm code
  // }
}
