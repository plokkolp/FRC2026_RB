package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConsClimber;

public class Climber extends SubsystemBase {

  private final TalonFX left  = new TalonFX(ConsClimber.LEFT_CLIMBER_ID);
  private final TalonFX right = new TalonFX(ConsClimber.RIGHT_CLIMBER_ID);
//-47.8 //-110
  private final PositionVoltage posRequest = new PositionVoltage(0).withSlot(0);

  public Climber() {
    left.getConfigurator().apply(ConsClimber.LEFT_CONFIG);
    right.getConfigurator().apply(ConsClimber.RIGHT_CONFIG);
  }

    public void setDoubleSpeed(double speed) {
    left.setControl(new DutyCycleOut(speed));
    right.setControl(new DutyCycleOut(speed));
  }

  public void setPosition(double rotations) {
    left.setControl(posRequest.withPosition(rotations));
    right.setControl(posRequest.withPosition(rotations));
  }

  public double getLeftPosition() {
    return left.getPosition().getValueAsDouble();
  }

  public double getRightPosition() {
    return right.getPosition().getValueAsDouble();
  }

  public double getAveragePosition() {
    return (getLeftPosition() + getRightPosition()) / 2.0;
  }

  public void resetPosition(double rot) {
    left.setPosition(rot);
    right.setPosition(rot);
  }

   @Override
  public void periodic() {
    SmartDashboard.putNumber("climber/ALL", getAveragePosition());
    SmartDashboard.putNumber("climber/Right", getRightPosition());
    SmartDashboard.putNumber("climber/Left", getLeftPosition());

  }
}