package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {

  private final Climber climber;
  private final double speed;
  private final XboxController controller;

  public Climb(Climber climber, double speed, XboxController controller) {
    this.climber = climber;
    this.speed = speed;
    this.controller = controller;
    addRequirements(climber);
  }

@Override
  public void initialize() {
   climber.setPosition(-47.8);

  }

  @Override
  public void execute() {
    
    if(controller.getBButton()){
       climber.setPosition(-110);
    }else{
      climber.setPosition(-47.8);
    }
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