// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autonomous;
import frc.robot.commands.resetPoseByTag;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;


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

private final SendableChooser<Command> autoChooser;
private final TagFollower tagFollower =
    new TagFollower(swerve);
    
    Pose2d speakerPose = new Pose2d(
    1.85,
    5.50,
    Rotation2d.fromDegrees(180)
);

    Pose2d initialPose = new Pose2d(
      0.0,
      0.0,
      Rotation2d.fromDegrees(0)
    );
    

   Pose2d EndGamePose = new Pose2d(
    9.15,
    5.65,
   Rotation2d.fromDegrees(0)
   );

  public RobotContainer() {

    swerve.configureAutoBuilder();
    NamedCommands.registerCommand("ResetWithMegaTag2", ResetPoseByTag);
    NamedCommands.registerCommand("FollowTag",tagFollower);

      
    if (AutoBuilder.isConfigured()) {
      autoChooser = AutoBuilder.buildAutoChooser();
      System.out.println("✅ AutoChooser criado");
  } else {
      System.out.println("❌ AutoBuilder NÃO configurado, AutoChooser vazio");
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
            () -> ClimbSub.setMotor(-0.5),
            () -> ClimbSub.STOP(),
            ClimbSub
        )
    );
new Trigger(ps5::getL2Button)
.whileTrue(
  Commands.startEnd(
      () -> ClimbSub.setMotor(0.5),
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
      () -> GoToPoseCommand.go(speakerPose),
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

  new Trigger(ps5::getL1Button)
  .whileTrue(
    Commands.startEnd(
      () -> shooterSub.DescobrirKV(),
      () ->shooterSub.StopShooter(),
    shooterSub
    ));

    new Trigger(ps5::getR1Button)
    .whileTrue(
      Commands.deadline(
        ShooterCommand,
        new RunCommand(() -> swerve.SnapToTag(), swerve),
        Commands.startEnd(
          () -> armazenamento.setMotorArmazenamento(0.5),
          armazenamento::StopMotorArmazenamento,
          armazenamento
        )
      )
    );
  }
  
  
  
  public Command getAutonomousCommand() {
    swerve.resetOdometry(initialPose);
    return autoChooser.getSelected();
}

}
