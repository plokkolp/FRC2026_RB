package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberTest extends Command {

  private final Climber climber;
  private final double speed;

  public ClimberTest(Climber climber, double speed) {
    this.climber = climber;
    this.speed = speed;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    climber.setDoubleSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    climber.setDoubleSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}