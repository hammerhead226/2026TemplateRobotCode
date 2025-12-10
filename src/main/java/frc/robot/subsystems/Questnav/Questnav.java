package frc.robot.subsystems.Questnav;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Questnav extends SubsystemBase {
  private final QuestnavIO IO;
  private final QuestnavIOInputsAutoLogged input = new QuestnavIOInputsAutoLogged();

  public Questnav(QuestnavIO IO) {
    this.IO = IO;
  }

  @Override
  public void periodic() {
    IO.commandPeriodic();
    IO.updateInputs(input);
    Logger.processInputs("headset", input);
    // add alerts

  }

  public boolean isTrustworthy() {
    return (input.isConnected && input.latency < 10 && input.frameCount > 60 && input.tracking);
  }
}
