package frc.robot.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotTelemetry;
import frc.robot.autonomous.modes.AutoModeBase;
import frc.robot.autonomous.modes.CenterMode;
import frc.robot.autonomous.modes.DoNothingMode;
import frc.robot.autonomous.modes.LeftMode;
import frc.robot.autonomous.modes.RightMode;
import frc.robot.autonomous.modes.TestMode;
import frc.robot.autonomous.tasks.Task;
import frc.robot.subsystems.leds.LEDModes;
import frc.robot.subsystems.leds.LEDs;

public class AutoRunner {
  private static AutoRunner m_autoRunner = null;
  private AutoChooser m_autoChooser;
  private AutoModeBase m_autoMode;
  private LEDs m_leds;

  public enum AutoMode {
    DO_NOTHING,
    DEFAULT,
    TEST,
    LEFT,
    RIGHT,
    CENTER
  }

  private AutoRunner() {
    // Use this to set the default auto mode
    AutoMode defaultAuto = AutoMode.TEST; // TODO: maybe change this
    m_leds = LEDs.getInstance();

    m_autoChooser = AutoChooser.getInstance();

    // Sets the default auto to the mode from above, then listens for changes
    // to the auto chooser and updates the selected auto mode
    onAutoChange(defaultAuto.toString());
    m_autoChooser.setDefaultOption(defaultAuto.toString());
    m_autoChooser.setOnChangeCallback(this::onAutoChange);
  }

  private AutoMode m_selectedAuto;

  public static AutoRunner getInstance() {
    if (m_autoRunner == null) {
      m_autoRunner = new AutoRunner();
    }
    return m_autoRunner;
  }

  public Task getNextTask() {
    return m_autoMode.getNextTask();
  }

  public void initialize() {
    onAutoChange(m_selectedAuto.toString());
  }

  private void onAutoChange(String newAuto) {
    // Color color = DriverStation.getAlliance().get().equals(Alliance.Blue) ? Color.kBlue : Color.kRed;
    // Alliance alliance = DriverStation.getAlliance().get();
    // Color color;
    // if(alliance != null) {
    //   color = alliance.equals(Alliance.Blue) ? Color.kBlue : Color.kRed;
    // } else {
    //   color = Color.kBlue;
    // }
    Color color = Color.kRed;
    RobotTelemetry.print("AUTO CHANGED");
    m_selectedAuto = AutoMode.valueOf(newAuto);
    RobotTelemetry.print(m_selectedAuto.toString());

    switch (m_selectedAuto) {
      case DO_NOTHING -> {
        m_leds.setAllColor(Color.kBlack);
        m_autoMode = new DoNothingMode();
      }
      case TEST -> {
        m_leds.setAllColorMode(LEDModes.rainbowChase);
        m_autoMode = new TestMode();
      }
      case LEFT -> {
        m_leds.setRightColor(color);
        m_leds.setLeftColor(Color.kBlack);
        m_autoMode = new LeftMode();
      }
      case RIGHT -> {
        m_leds.setLeftColor(color);
        m_leds.setRightColor(Color.kBlack);
        m_autoMode = new RightMode();
      }
      case CENTER -> {
        m_leds.setAllColor(color);
        m_autoMode = new CenterMode();
      }
      default -> {
        RobotTelemetry.print("Invalid auto mode selected. Defaulting to do nothing.");
        m_autoMode = new DoNothingMode();
      }
    }

    m_autoMode.queueTasks();
  }
}
