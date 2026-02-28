package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.Auto.AutoClimber;
import frc.robot.commands.Auto.AutoGetFuelDown;
import frc.robot.commands.Auto.AutoGetFuelUp;
import frc.robot.commands.Auto.AutoShoot;
import frc.robot.constants.ConsController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final XboxController driver = new XboxController(ConsController.kDriveControllerPort);
  private final XboxController testJoy = new XboxController(3);
  private final XboxController operator = new XboxController(ConsController.kOperatorControllerPort);
  private final XboxController TT = new XboxController(4);

  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Climber m_Climber = new Climber();
  private final LL4 m_Ll4 = new LL4("limelight-shoot");

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final Vision vision = new Vision(drivetrain);

  private final SendableChooser<Command> autoChooser;

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
  private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

  public RobotContainer(Supplier<Boolean> m_isRedAlliance) {
    drivetrain.setAllianceSupplier(m_isRedAlliance);
    configureBindings();

    configureNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {
    setDefaultCommand();
    Trigger leftTrigger = new Trigger(() -> testJoy.getRawAxis(ConsController.Axis.LEFT_TRIGGER.id) > 1);

    // new JoystickButton(driver, ConsController.Button.BUTTON_LB.id)
    //     .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // new JoystickButton(driver, ConsController.Button.BUTTON_LB.id)
    // .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // new JoystickButton(driver, ConsController.Button.BUTTON_A.id)
    // .whileTrue(drivetrain.applyRequest(() -> brake));

    // new JoystickButton(driver, ConsController.Button.BUTTON_LB.id)
    // .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // new JoystickButton(driver, ConsController.Button.BUTTON_Y.id)
    // .toggleOnTrue(new ShooterMaster(m_shooter));
//====================================================================//
    
    // new JoystickButton(driver, ConsController.Button.BUTTON_Y.id)
    //     .toggleOnTrue(new Shoot2(m_shooter, operator, driver));

    // new JoystickButton(driver, ConsController.Button.BUTTON_B.id)
    //     .toggleOnTrue(new Climb(m_Climber, driver));

    // new JoystickButton(driver, ConsController.Button.BUTTON_RB.id)
    //     .toggleOnTrue(new FuelGoBack(m_shooter, driver));
   
    new JoystickButton(driver,ConsController.Button.BUTTON_C.id)
      .onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(
        drivetrain::seedFieldCentric, drivetrain));
//
    new JoystickButton(operator, ConsController.Button.BUTTON_RB.id)
        .toggleOnTrue(new GetFuelDown(m_intake, operator));

    new JoystickButton(operator, ConsController.Button.BUTTON_LB.id)
        .toggleOnTrue(new GetFuelUp(m_intake));
      
//====================================================================//

    new JoystickButton(testJoy, ConsController.Button.BUTTON_B.id)
            .toggleOnTrue(new ClimberTest( m_Climber,0.2));

    new JoystickButton(testJoy, ConsController.Button.BUTTON_X.id)
            .toggleOnTrue(new ClimberTest( m_Climber,-0.1));
//====================================================================//
    
    // new JoystickButton(driver, ConsController.Button.BUTTON_Y.id)
    // .whileTrue(new ShooterEasy(m_shooter, driver
    // ));
    // new JoystickButton(testJoy, ConsController.Button.BUTTON_A.id)
    // .onTrue(edu.wpi.first.wpilibj2.command.Commands.print("A pressed"));
    
// new JoystickButton(driver, ConsController.Button.BUTTON_RB.id)
//     .onTrue(new AlignTheta(drivetrain,+12));

new JoystickButton(driver, ConsController.Button.BUTTON_LB.id)
    .onTrue(new AlignToPose(drivetrain));
  }

  private void setDefaultCommand() {

    // drivetrain.setDefaultCommand(
    // new Drive(
    // drivetrain,
    // () -> -driver.getRawAxis(ConsController.Axis.LEFT_STICK_Y.id) * maxSpeed,
    // () -> +driver.getRawAxis(ConsController.Axis.LEFT_STICK_X.id) * maxSpeed,
    // () -> -driver.getRawAxis(ConsController.Axis.RIGHT_STICK_X.id) *
    // maxAngularRate
    // )
    // );
    drivetrain.setDefaultCommand(
        new Drive(
            drivetrain,
            () -> -MathUtil.applyDeadband(-driver.getRawAxis(ConsController.Axis.LEFT_STICK_Y.id), 0.06),
            () -> +MathUtil.applyDeadband(+driver.getRawAxis(ConsController.Axis.LEFT_STICK_X.id), 0.06),
            () -> +MathUtil.applyDeadband(-driver.getRawAxis(ConsController.Axis.RIGHT_STICK_X.id), 0.06)));

    // m_shooter.setDefaultCommand(
    //     new ShooterTest(
    //         m_shooter,
    //         () -> MathUtil.applyDeadband(
    //             -testJoy.getRawAxis(ConsController.Axis.RIGHT_STICK_Y.id),
    //             0.1),
    //         () -> testJoy.getRawAxis(ConsController.Axis.RIGHT_TRIGGER.id)));

    // m_intake.setDefaultCommand(
    //     new IntakeTest(
    //         m_intake,
    //         () -> TT.getRawAxis(ConsController.Axis.RIGHT_TRIGGER.id),
    //         () -> TT.getRawAxis(ConsController.Axis.LEFT_TRIGGER.id),
    //         () -> TT.getRawAxis(ConsController.Axis.RIGHT_STICK_Y.id)));

    // m_shooter.setDefaultCommand(
    // new ShooterTest(
    // m_shooter,
    // () -> MathUtil.applyDeadband(
    // -testJoy.getRawAxis(ConsController.Axis.RIGHT_STICK_Y.id),
    // 0.1
    // )
    // )
    // );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Shoot", new MJ(m_shooter));
    NamedCommands.registerCommand("Shoot1", new MJ1(m_shooter));
    NamedCommands.registerCommand("Shooter", new AutoShoot(m_shooter));
    NamedCommands.registerCommand("Climber", new AutoClimber(m_Climber));
    NamedCommands.registerCommand("UP", new AutoGetFuelUp(m_intake));
    NamedCommands.registerCommand("DOWN", new AutoGetFuelDown(m_intake));

  }
}
