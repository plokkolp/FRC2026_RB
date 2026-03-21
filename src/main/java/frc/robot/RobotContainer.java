package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.Auto.AutoGetFuelDown;
import frc.robot.commands.Auto.AutoGetFuelUp;
import frc.robot.commands.Auto.AutoShoot;
import frc.robot.constants.ConsController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {

  private final XboxController driver = new XboxController(ConsController.kDriveControllerPort);
  // private final XboxController testJoy = new XboxController(3);
  private final XboxController operator = new XboxController(ConsController.kOperatorControllerPort);
  // private final XboxController TT = new XboxController(4);

  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Climber m_Climber = new Climber();
  private final LL4 m_Ll4 = new LL4("limelight-shoot");

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final Vision vision = new Vision(drivetrain);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer(Supplier<Boolean> m_isRedAlliance) {
    drivetrain.setAllianceSupplier(m_isRedAlliance);
    configureBindings();

    configureNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {
    setDefaultCommand();
    // Trigger leftTrigger = new Trigger(() -> testJoy.getRawAxis(ConsController.Axis.LEFT_TRIGGER.id) > 1);
    
    new JoystickButton(driver, ConsController.Button.BUTTON_RB.id)
        .toggleOnTrue(new Shoot2Drive(m_shooter, drivetrain, operator, driver));

    new JoystickButton(driver, ConsController.Button.BUTTON_Y.id)
        .toggleOnTrue(new Shoot2Static(m_shooter, operator, driver));

    new JoystickButton(driver, ConsController.Button.BUTTON_B.id)
        .toggleOnTrue(new Climb(m_Climber, driver));

    new JoystickButton(driver, ConsController.Button.BUTTON_LB.id)
         .toggleOnTrue(new FUELGOBACK(m_shooter,drivetrain,driver));
   
    new JoystickButton(driver,ConsController.Button.BUTTON_Z.id)
      .onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(
        drivetrain::seedFieldCentric, drivetrain));
// 
    new JoystickButton(operator, ConsController.Button.BUTTON_RB.id)
        .toggleOnTrue(new GetFuelDown(m_intake, operator,driver));

    new JoystickButton(operator, ConsController.Button.BUTTON_LB.id)
        .toggleOnTrue(new GetFuelUp(m_intake));

      
//====================================================================//

    // new JoystickButton(testJoy, ConsController.Button.BUTTON_B.id)
    //         .whileTrue(new ClimberTest( m_Climber,0.4));

    // new JoystickButton(testJoy, ConsController.Button.BUTTON_X.id)
    //         .whileTrue(new ClimberTest( m_Climber,-0.5));
//====================================================================//  
  }

  private void setDefaultCommand() {

    drivetrain.setDefaultCommand(
        new Drive(
            drivetrain,
            () -> +MathUtil.applyDeadband(-driver.getRawAxis(ConsController.Axis.LEFT_STICK_Y.id), 0.06),
            () -> -MathUtil.applyDeadband(+driver.getRawAxis(ConsController.Axis.LEFT_STICK_X.id), 0.06),
            () -> +MathUtil.applyDeadband(-driver.getRawAxis(ConsController.Axis.RIGHT_STICK_X.id), 0.06)));

    // m_intake.setDefaultCommand(
    //     new IntakeTest(
    //         m_intake,m_shooter,                                         //灌程式 搖桿4號
    //         () -> TT.getRawAxis(ConsController.Axis.LEFT_STICK_Y.id),   //INTAKE上下角度
    //         () -> -TT.getRawAxis(ConsController.Axis.RIGHT_TRIGGER.id), //按右邊的板機(寫R2的那個) 兩個傳輸加砲台一起轉
    //         () -> TT.getRawAxis(ConsController.Axis.RIGHT_STICK_Y.id)));//純INTAKE 管旋轉
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureNamedCommands() {
    
    NamedCommands.registerCommand("Shooter", new AutoShoot(m_shooter));
    NamedCommands.registerCommand("UP", new AutoGetFuelUp(m_intake));
    NamedCommands.registerCommand("DOWN", new AutoGetFuelDown(m_intake));
    

  }
}
