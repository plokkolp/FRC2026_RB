package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoGetFuelDown extends Command {

  private final Intake intake;

  private static final double POS_DOWN = 14.81;
  private static final double TOLERANCE = 1.0;

  public AutoGetFuelDown(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setAnglePositionRot(POS_DOWN);
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
    return false;
  }
}