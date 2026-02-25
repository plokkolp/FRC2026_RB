package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class TeleGetFuel extends Command {

  private final Intake intake;
    private final XboxController controller;


  private static final double POS_DOWN = 14.81;
  private static final double POS_UP   = 0.0;
  private static final double TOLERANCE = 1;

  private boolean retracting; 

  public TeleGetFuel(Intake intake,XboxController controller) {
    this.intake = intake;
    this.controller = controller;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    double current = intake.getAnglePositionRot();

    if (Math.abs(current - POS_DOWN) < TOLERANCE) {
      retracting = true;
      intake.setAnglePositionRot(POS_UP);
      intake.setRolling(0.9);     
    } 
    else {
      retracting = false;
      intake.setAnglePositionRot(POS_DOWN);
      intake.setRolling(0.9);    
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    intake.setRolling(0.0);
  }

  @Override
  public boolean isFinished() {
    if (!retracting) {
      return false;
    }

    double current = intake.getAnglePositionRot();
    return Math.abs(current - POS_UP) < 1.2;
  }
}