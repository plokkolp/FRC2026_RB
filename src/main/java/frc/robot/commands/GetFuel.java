package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class GetFuel extends Command {

  private final Intake intake;

  private static final double POS_DOWN = 12.0;
  private static final double POS_UP   = 0.0;

  private static final double TOLERANCE = 0.5;

   private static boolean down = false, end = false;
   
     public GetFuel(Intake intake) {
       this.intake = intake;
       addRequirements(intake);
     }
   
     @Override
     public void initialize() {
        down = false; end = false;
       double currentPos = intake.getAnglePositionRot(); 
   
       if (Math.abs(currentPos - POS_DOWN) < TOLERANCE) {
         intake.setAnglePositionRot(POS_UP);
         
    }
    else {
      intake.setAnglePositionRot(POS_DOWN);
     
    }
  }

@Override
  public void execute() {
   

    if(Math.abs(intake.getAnglePositionRot() - POS_DOWN) <= 1 ){
        intake.setRolling(1);
    }else{
         intake.setRolling(0);
    }

  }

   @Override
  public void end(boolean interrupted) {
    intake.setRolling(0);
  }
  
  @Override
  public boolean isFinished() {
    return end;
  }
}
