package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class AutoClimber extends Command {

  private final Climber climber;

  public AutoClimber(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

@Override
  public void initialize() {
   climber.setPosition(-47.8);
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
   climber.setPosition(-135);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}