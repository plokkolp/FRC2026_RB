package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class GetFuelDown extends Command {

  private final Intake intake;
  private final XboxController controller;

  private static final double POS_DOWN = 14.81;

  public GetFuelDown(Intake intake, XboxController controller) {
    this.intake = intake;
    this.controller = controller;
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