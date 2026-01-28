package frc.robot;

import frc.robot.commands.resetPoseByTag;
import frc.robot.commands.snapToTag;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.GoToPoseCommand;
import frc.robot.commands.Loc;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TagFollower;
import frc.robot.commands.TesteSwerveMotors;
import frc.robot.subsystems.Armazenamento;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Set;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


public class RobotContainer {

 private final SwerveSub swerve = new SwerveSub();
 private final Loc loc;
 private final PS5Controller ps5 = new PS5Controller(0);
 private final ClimbSub ClimbSub = new ClimbSub();
 private final TesteSwerveMotors testeSwerveMotors = new TesteSwerveMotors(swerve);
 private final resetPoseByTag ResetPoseByTag = new resetPoseByTag(swerve);
 private final ShooterSub shooterSub = new ShooterSub(swerve);
 private final ShooterCommand ShooterCommand = new ShooterCommand(shooterSub);
 private final IntakeSub Intake = new IntakeSub();
 private final Armazenamento armazenamento = new Armazenamento();
 private final snapToTag snapToTag = new snapToTag(swerve, shooterSub);

private final SendableChooser<Command> autoChooser;
private final TagFollower tagFollower =
    new TagFollower(swerve);
    
    Pose2d NeutralZone = new Pose2d(
    7.70,
    1.31,
    Rotation2d.fromDegrees(90)
);

    Pose2d initialPose = new Pose2d(
      2.0,
      7.0,
      Rotation2d.fromDegrees(0)
    );
    

   Pose2d EndGamePose = new Pose2d(
    1.5,
    3.5,
   Rotation2d.fromDegrees(150)
   );

  public RobotContainer() {

    NamedCommands.registerCommand("ResetWithMegaTag2", ResetPoseByTag);
    NamedCommands.registerCommand("FollowTag",tagFollower);
    NamedCommands.registerCommand("IntakeAngOn", new RunCommand(() -> Intake.setIntakeAngle(60), Intake));
    NamedCommands.registerCommand("ShooterOn", ShooterCommand);
    NamedCommands.registerCommand("IntakeRotOn", new RunCommand(() -> Intake.setIntakeRotSpeed(0.7), Intake));
    NamedCommands.registerCommand("IntakeRotOff", new RunCommand(() -> Intake.setIntakeRotSpeed(0.0), Intake));
    NamedCommands.registerCommand("ClimbAuto", new ClimbCommand(ClimbSub, 0.50));

         
    swerve.configureAutoBuilder();

    if (AutoBuilder.isConfigured()) {
      autoChooser = AutoBuilder.buildAutoChooser();
      System.out.println(" AutoChooser criado");
  } else {
      System.out.println("AutoBuilder NÃO configurado, AutoChooser vazio");
      autoChooser = new SendableChooser<>();
  }
  
  SmartDashboard.putData("Auto Mode", autoChooser);
    
    loc = new Loc(swerve,ps5);
    swerve.setDefaultCommand(loc);

    configurationBindings();
  }

  public void configurationBindings() {
   
    new Trigger(ps5::getR2Button)
    .whileTrue(
        Commands.startEnd(
            () -> ClimbSub.setMotor(-0.1),
            () -> ClimbSub.STOP(),
            ClimbSub
        )
    );
new Trigger(ps5::getL2Button)
.whileTrue(
  Commands.startEnd(
      () -> ClimbSub.setMotor(0.1),
      () -> ClimbSub.STOP(),
      ClimbSub
  )
);

new Trigger(ps5::getCircleButton)
.onTrue(new ClimbCommand(ClimbSub, 0.50));

new Trigger(ps5::getCrossButton)
.onTrue(new ClimbCommand(ClimbSub, 0.0));

new Trigger(ps5::getTriangleButton)
.whileTrue(tagFollower);

new Trigger(ps5::getSquareButton)
  .toggleOnTrue(
    Commands.defer(
      () -> GoToPoseCommand.go(NeutralZone),
      Set.of(swerve)
    )
  );
  new Trigger(ps5::getTouchpadButton)
  .onTrue(
    Commands.sequence(
      ResetPoseByTag,
      GoToPoseCommand.go(EndGamePose)
    )
  );

  new Trigger(ps5::getOptionsButton)
  .toggleOnTrue(testeSwerveMotors);

  /*new Trigger(ps5::getL1Button)
  .whileTrue(
    Commands.startEnd(
      () -> shooterSub.DescobrirKV(),
      () ->shooterSub.StopShooter(),
    shooterSub
    )); /*
    
    */

    new Trigger(ps5::getR1Button)
    .whileTrue(
      Commands.deadline(
        ShooterCommand,
         snapToTag,
        Commands.startEnd(
          () -> armazenamento.setMotorArmazenamento(0.5),
          armazenamento::StopMotorArmazenamento,
          armazenamento
        )
      )
    );

  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
}

}
