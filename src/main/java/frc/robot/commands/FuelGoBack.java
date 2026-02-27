package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class FuelGoBack extends Command {

  private final Shooter m_shooter;
  private final XboxController controller;

  public FuelGoBack(Shooter m_shooter,XboxController controller) {
    this.m_shooter = m_shooter;
    this.controller = controller;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() { 
  }

  @Override
  public void execute() {
    m_shooter.setYawAbsRot(0.0);
    m_shooter.setShooterRPM(3800);
    
    if(controller.getRightTriggerAxis() > 0.3){
      m_shooter.setPitchPosition(-2.11);
      
      if(m_shooter.getPitchPositionRot() < -1.7){
      m_shooter.setTrainSpeed(-0.7);
      m_shooter.setIntaketrainSpeed(-0.7);
      }

    } else if(controller.getLeftTriggerAxis() > 0.3){
           
      m_shooter.setTrainSpeed(0.7);
    }else{
      
      m_shooter.setPitchPosition(-0.25);
      m_shooter.setTrainSpeed(0);
      m_shooter.setIntaketrainSpeed(0);
    }

  }

   @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_shooter.stopAngle();
    m_shooter.setTrainSpeed(0);
    m_shooter.setIntaketrainSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}