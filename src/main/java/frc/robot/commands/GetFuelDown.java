package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class GetFuelDown extends Command {

  private final Intake intake;
  private final XboxController controller;
    private final XboxController driver;

  private static final double POS_up = 5.9765625;

  private static final double POS_DOWN = 14.83;

  public GetFuelDown(Intake intake, XboxController controller,XboxController driver) {
    this.intake = intake;
    this.controller = controller;
    this.driver = driver;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setAnglePositionRot(POS_DOWN);   
    intake.setRolling(0.9);              
  }

  @Override
  public void execute() {
    if (controller.getAButton()) {
      intake.setRolling(-0.92);             
    } else {
      intake.setRolling(0.9);            
    }
    
    if (driver.getRightTriggerAxis() > 0.95) {
      intake.setAnglePositionRot(POS_up);   
    } else {
      intake.setAnglePositionRot(POS_DOWN);   
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.setRolling(0.9);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}