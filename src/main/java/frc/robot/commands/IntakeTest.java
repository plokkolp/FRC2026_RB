package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier; 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeTest extends Command {
  private final Intake intake;
  private final DoubleSupplier up;     
  private final DoubleSupplier down;   
  private final DoubleSupplier isAPressed; 

  public IntakeTest(Intake intake, DoubleSupplier up, DoubleSupplier down, DoubleSupplier isAPressed) {
    this.intake = intake;
    this.up = up;
    this.down = down;
    this.isAPressed = isAPressed;

    addRequirements(intake);
  }

  @Override
  public void execute() {
    double rollingSpeed = isAPressed.getAsDouble();
    intake.setRolling(rollingSpeed);

    double anglespeed = (up.getAsDouble() - down.getAsDouble()) / 4.0;
    intake.setAngleSpeed(anglespeed); 
  }

  @Override
  public void end(boolean interrupted) {
    intake.setRolling(0);
    intake.setAngleSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}