package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class GetFuelUp extends Command {

  private final Intake intake;

  private static final double POS_UP = 0.0;
  private static final double TOLERANCE = 1.2;

  public GetFuelUp(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setAnglePositionRot(POS_UP); 
    intake.setRolling(0.9);            
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
    double current = intake.getAnglePositionRot();
    return Math.abs(current - POS_UP) < TOLERANCE;
  }
}