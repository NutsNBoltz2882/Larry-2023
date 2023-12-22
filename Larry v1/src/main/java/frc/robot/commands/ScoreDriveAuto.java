// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreDriveAuto extends SequentialCommandGroup {
  /** Creates a new ScoreDriveAuto. */
  public ScoreDriveAuto(SwerveSubsystem swerveSubsystem) {
    addRequirements(swerveSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(3.0, 0, 0)))).withTimeout(.2),
      new InstantCommand(() -> swerveSubsystem.stopModules()),
      Commands.waitSeconds(.2),
      new RunCommand(() -> swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(-2.0, 0, 0)))).withTimeout(3.5),
      new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
}
