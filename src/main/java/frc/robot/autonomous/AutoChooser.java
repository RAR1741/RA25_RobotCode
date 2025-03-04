package frc.robot.autonomous;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.AutoRunner.AutoMode;

public class AutoChooser {
  AutoMode m_selectedAuto;

  private static AutoChooser m_instance = null;

  private String m_selectedAutoName;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private AutoChooser() {
    m_chooser.setDefaultOption("TEST", "TEST"); // TODO: This should be DO_NOTHING or whatever we are defaulting to for auto

    // Populate the chooser with all the available autos
    for (AutoMode mode : AutoRunner.AutoMode.values()) {
      m_chooser.addOption(mode.name(), mode.name());
    }

    SmartDashboard.putData("Auto picker", m_chooser);
  }

  public static AutoChooser getInstance() {
    if(m_instance == null) {
      m_instance = new AutoChooser();
    }
    return m_instance;
  }

  private void updateSelectedAuto() {
    m_selectedAutoName = m_chooser.getSelected();
    m_selectedAuto = AutoRunner.AutoMode.valueOf(m_selectedAutoName);
  }

  public AutoMode getSelectedAuto() {
    updateSelectedAuto();
    return m_selectedAuto;
  }

  public void setDefaultOption(String option) {
    m_chooser.setDefaultOption(option, option);
  }

  public void setOnChangeCallback(Consumer<String> listener) {
    m_chooser.onChange(listener);
  }
}
