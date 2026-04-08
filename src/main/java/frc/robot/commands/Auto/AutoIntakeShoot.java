package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntakeShoot extends Command {

  private final Intake intake;

  private static final double POS_up = 6.1;

  private static final double POS_DOWN = 15.61474609375;

  public AutoIntakeShoot(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  
  @Override
  public void initialize() {
    intake.setRolling(0.9);              
  }

  @Override
  public void execute() {
    
    if (intake.getAnglePositionRot() > 14) {
      intake.setAnglePositionRot(POS_up);   
    } else if (intake.getAnglePositionRot() < 6.5){
      intake.setAnglePositionRot(POS_DOWN);   
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.setRolling(0);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}