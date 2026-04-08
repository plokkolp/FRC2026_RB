package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {

  private final Climber climber;
  private final XboxController controller;

  public Climb(Climber climber, XboxController controller) {
    this.climber = climber;
    this.controller = controller;
    addRequirements(climber);
  }

@Override
  public void initialize() {
   climber.setPosition(-88.8);

  }

  @Override
  public void execute() {
    
    if(controller.getXButtonPressed()){
       climber.setPosition(-188);
    }
  }

  @Override
  public void end(boolean interrupted) {
      climber.setPosition(-1.8);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}