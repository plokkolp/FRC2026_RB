package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier; 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeTest extends Command {
  private final Intake intake;
  private final DoubleSupplier up;     // 改名以便閱讀：對應右手板機
  private final DoubleSupplier down;   // 改名以便閱讀：對應左手板機
  private final BooleanSupplier isAPressed; // 控制滾動

  public IntakeTest(Intake intake, DoubleSupplier up, DoubleSupplier down, BooleanSupplier isAPressed) {
    this.intake = intake;
    this.up = up;
    this.down = down;
    this.isAPressed = isAPressed;

    addRequirements(intake);
  }

  @Override
  public void execute() {
    double rollingSpeed = isAPressed.getAsBoolean() ? 1.0 : 0.0;
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