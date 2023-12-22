package frc.robot;

import java.util.List;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BalanceAuto;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.GyroBalance;
import frc.robot.commands.ScoreDriveAuto;
import frc.robot.commands.StillAuto;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.SwerveRotateDrive;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final Extension extendSubsystem = new Extension();
        private final Lift liftSubsystem = new Lift();
        private final Grabber grabSubsystem = new Grabber();
        private final ColorSensor colorSensor = new ColorSensor();

        private final SendableChooser<Command> m_chooser = new SendableChooser<>();
        private final DriveAuto driveAuto = new DriveAuto(swerveSubsystem);
        private final ScoreDriveAuto scoreAuto = new ScoreDriveAuto(swerveSubsystem);
        private final BalanceAuto balanceAuto = new BalanceAuto(swerveSubsystem);
        private final GyroBalance gyroAuto = new GyroBalance(swerveSubsystem);
        private final StillAuto stillAuto = new StillAuto(swerveSubsystem);

        // Thread m_visionThread;
        private final CommandXboxController driverController = new CommandXboxController(0);
        private final CommandXboxController armController = new CommandXboxController(1);

        public RobotContainer() {
                extendSubsystem.resetEncoders();
                liftSubsystem.resetEncoders();
                CameraServer.startAutomaticCapture();
                liftSubsystem.setDefaultCommand(new RunCommand(
                                () -> liftSubsystem.adjust(() -> armController.getLeftY() * -1), liftSubsystem));
                extendSubsystem.setDefaultCommand(new RunCommand(
                                () -> extendSubsystem.adjust(() -> armController.getRightY() * -1), extendSubsystem));
                // colorSensor.setDefaultCommand(colorSensor.output());
                // m_visionThread =
                // new Thread(() -> {
                // // Get the UsbCamera from CameraServer
                // UsbCamera camera = CameraServer.startAutomaticCapture();
                // // Set the resolution
                // camera.setResolution(640, 480);
                // // Get a CvSink. This will capture Mats from the camera
                // CvSink cvSink = CameraServer.getVideo();
                // // Setup a CvSource. This will send images back to the Dashboard
                // CvSource outputStream = CameraServer.putVideo("Grabber", 160, 120);
                // // Mats are very memory expensive. Lets reuse this Mat.
                // Mat mat = new Mat();
                // // This cannot be 'true'. The program will never exit if it is. This
                // // lets the robot stop this thread when restarting robot code or deploying.
                // while (!Thread.interrupted()) {
                // // Tell the CvSink to grab a frame from the camera and put it
                // // in the source mat. If there is an error notify the output.
                // if (cvSink.grabFrame(mat) == 0) {
                // // Send the output the error.
                // outputStream.notifyError(cvSink.getError());
                // // skip the rest of the current iteration
                // continue;}
                // // Put a rectangle on the image
                // Imgproc.rectangle(mat, new Point(10, 100), new Point(70, 100), new
                // Scalar(255, 255, 255), 5);
                // Imgproc.rectangle(mat, new Point(40, 70), new Point(40, 130), new Scalar(255,
                // 255, 255), 5);
                // // Give the output stream a new image to display
                // outputStream.putFrame(mat);
                // }});
                // m_visionThread.setDaemon(true);
                // m_visionThread.start();

                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> driverController.getLeftY(),
                                () -> driverController.getLeftX(),
                                () -> driverController.getRightX(),
                                () -> true));
                // () -> driverController.getHID().getStartButton()));

                m_chooser.setDefaultOption("Drive", driveAuto);
                m_chooser.addOption("Score", scoreAuto);
                m_chooser.addOption("Balance", balanceAuto);
                m_chooser.addOption("Gyro", gyroAuto);
                m_chooser.addOption("Still", stillAuto);
                SmartDashboard.putData("Autonomous", m_chooser);
                // SmartDashboard.putNumber("Front left drive",
                // SwerveSubsystem.getDrivePosition());

                configureButtonBindings();
        }

        private void configureButtonBindings() {
                driverController.start().onTrue(swerveSubsystem.zeroHeading());
                driverController.back().onTrue(swerveSubsystem.resetEncodersCommand());
                // driverController.rightBumper().whileTrue(new SwerveRotateDrive(
                // swerveSubsystem, () -> driverController.getLeftY(), () ->
                // driverController.getLeftX(), 0.0,
                // () -> true));
                // driverController.leftBumper().whileTrue(new SwerveRotateDrive(
                // swerveSubsystem, () -> driverController.getLeftY(), () ->
                // driverController.getLeftX(), 180.0,
                // () -> true));

                // driverController.b().whileTrue(-[6new StartEndCommand((() ->
                // swerveSubsystem.setModuleStates(
                // DriveConstants.kDriveKinematics.toSwerveModuleStates(
                // new ChassisSpeeds(-.5, 0, 0)))),
                // () -> swerveSubsystem.stopModules(), swerveSubsystem));

                // Trigger sensor = new Trigger(() -> colorSensor.pieceIn());
                // sensor.whileTrue(new StartEndCommand(() ->
                // armController.getHID().setRumble(RumbleType.kBothRumble, .5), () ->
                // armController.getHID().setRumble(RumbleType.kBothRumble, 0)));
                armController.rightBumper()
                                .onTrue(grabSubsystem.grabCommand().unless(() -> liftSubsystem.getPosition() < 9));
                // armController.leftTrigger(.07).whileTrue(new RunCommand(() ->
                // liftSubsystem.adjust(() -> armController.getLeftTriggerAxis() * -1)));
                // armController.rightTrigger(.07).whileTrue(new RunCommand(() ->
                // liftSubsystem.adjust(() -> armController.getRightTriggerAxis() * 1)));
                // Trigger leftStick = new Trigger(() -> Math.abs(driverController.getLeftY()) >
                // .1);
                // leftStick.whileTrue(new RunCommand(() -> liftSubsystem.adjust(() ->
                // armController.getLeftY() * -1), liftSubsystem));
                // armController.leftStick().whileTrue(new RunCommand(() ->
                // liftSubsystem.adjust(() -> armController.getLeftY() * -1), liftSubsystem));
                // armController.leftStick().
                // armController.rightStick().onTrue(extendSubsystem.setSpeed(() ->
                // armController.getRightY() * .2));

                // armController.a().onTrue(liftSubsystem.move(() -> 20.0) //angle for floor
                // .withTimeout(3)); // .until(() -> liftSubsystem.atSetpoint()) //
                // .andThen(extendSubsystem.move(() -> 4.5))
                // .withTimeout(3)); // .until(() -> extendSubsystem.atSetpoint())); //
                // .withTimeout(2));

                armController.a().onTrue(liftSubsystem.setSetpoint(() -> 15.0));

                // armController.b().onTrue(liftSubsystem.move(() -> 50.5) //angle for platform
                // .withTimeout(3)); // .until(() -> liftSubsystem.atSetpoint()) //
                // .andThen(extendSubsystem.move(() -> 4.5))
                // .withTimeout(3));
                armController.x().onTrue(liftSubsystem.setSetpoint(() -> 22.0));
                armController.b().onTrue(liftSubsystem.setSetpoint(() -> 48.5));

                // armController.y().onTrue(liftSubsystem.move(() -> 65.0) //angle for top row
                // .withTimeout(3)); // .until(() -> liftSubsystem.atSetpoint()) //
                // .withTimeout(2)
                // .andThen(extendSubsystem.move(() ->
                // Constants.ArmConstants.kExtendForwardLimit))
                // .withTimeout(3)); // .until(() -> extendSubsystem.atSetpoint())); //
                // .withTimeout(2));

                armController.y().onTrue(liftSubsystem.setSetpoint(() -> 55.0));

                armController.back().onTrue(extendSubsystem.setSetpoint(() -> -1.0));
                armController.start().onTrue(extendSubsystem.setSetpoint(() -> 8.0));

                armController.leftBumper().onTrue(extendSubsystem.setSpeed(() -> -.1) // reset extension drift
                                .withTimeout(1)
                                .andThen(() -> extendSubsystem.resetEncoders()));
        }

        public Command getAutonomousCommand() {
                return m_chooser.getSelected();
                // return Commands.runEnd(() -> swerveSubsystem.setModuleStates(
                // DriveConstants.kDriveKinematics.toSwerveModuleStates
                // (new ChassisSpeeds(-2.0, 0, 0))),
                // () -> swerveSubsystem.stopModules(), swerveSubsystem).withTimeout(2.72);
                // //3.75

                // return Commands.runEnd(() -> swerveSubsystem.setModuleStates(
                // DriveConstants.kDriveKinematics.toSwerveModuleStates
                // (new ChassisSpeeds(-1.5, -.4, 0))),
                // () -> swerveSubsystem.stopModules(), swerveSubsystem).withTimeout(3);

                // return Commands.sequence(liftSubsystem.setSetpoint(() -> 35.0).withTimeout(2)
                // //lift arm up horizontally
                // .andThen(grabSubsystem.grabCommand().withTimeout(1)) //drop cone
                // .andThen(liftSubsystem.setSetpoint(() -> 15.0)) //bring arm down to floor
                // level
                // .alongWith(new InstantCommand(() -> swerveSubsystem.setModuleStates //drive
                // backwards
                // (DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds
                // (-1, 0, 0))), swerveSubsystem)).withTimeout(3)
                // .andThen(() -> swerveSubsystem.stopModules())); //stop moving

                // return Commands.sequence(liftSubsystem.setSetpoint(() ->
                // 35.0).andThen(Commands.waitSeconds(2)) //lift arm up horizontally
                // .andThen(grabSubsystem.grabCommand()).andThen(Commands.waitSeconds(1)) //drop
                // cone
                // .andThen(liftSubsystem.setSetpoint(() ->
                // 15.0)).andThen(Commands.waitSeconds(1)) //bring arm down to floor level
                // .andThen(new InstantCommand(() -> swerveSubsystem.setModuleStates //drive
                // backwards
                // (DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds
                // (-1, 0, 0))), swerveSubsystem)).andThen(Commands.waitSeconds(3))
                // .andThen(() -> swerveSubsystem.stopModules()));

                // return Commands.sequence(liftSubsystem.setSetpoint(() -> 35.0) //lift arm up
                // .until(() ->liftSubsystem.atSetpoint()))
                // .andThen(grabSubsystem.grabCommand()) //drop cone
                // .andThen(Commands.waitSeconds(1))
                // .andThen(liftSubsystem.setSetpoint(() -> 15.0)) //bring arm down to floor
                // level
                // .until(() ->liftSubsystem.atSetpoint())
                // .andThen(new InstantCommand(() -> swerveSubsystem.setModuleStates //drive
                // backwards
                // (DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds
                // (-1, 0, 0))), swerveSubsystem))
                // .andThen(Commands.waitSeconds(3))
                // .andThen(() -> swerveSubsystem.stopModules());

                // return Commands.sequence(liftSubsystem.move(() -> 55.0),
                // Commands.waitSeconds(3), //lift arm up horizontally
                // grabSubsystem.grabCommand(), Commands.waitSeconds(1), //drop cone
                // liftSubsystem.move(() -> 10.0), Commands.waitSeconds(2), //bring arm down to
                // floor level
                // new InstantCommand(() -> swerveSubsystem.setModuleStates //drive backwards
                // (DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds
                // (-1, 0, 0))), swerveSubsystem)).andThen(Commands.waitSeconds(5))
                // .andThen(() -> swerveSubsystem.stopModules());

                // liftSubsystem.setSetpoint(() -> 35.0).withTimeout(2),
                // grabSubsystem.grabCommand().withTimeout(1),
                // liftSubsystem.setSetpoint(() -> 15.0),
                // swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new
                // ChassisSpeeds(-4, 0, 0)))

                // .alongWith(new Commands.r(() -> swerveSubsystem.setModuleStates(
                // DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(-4, 0,
                // 0))))), swerveSubsystem);
                // // 1. Create trajectory settings
                // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                // AutoConstants.kMaxSpeedMetersPerSecond,
                // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // .setKinematics(DriveConstants.kDriveKinematics);

                // // 2. Generate trajectory
                // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                // new Pose2d(0, 0, new Rotation2d(0)),
                // List.of(
                // new Translation2d(1, 0),
                // new Translation2d(1, -1)),
                // new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                // trajectoryConfig);

                // // 3. Define PID controllers for tracking trajectory
                // PIDController xController = new PIDController(AutoConstants.kPXController, 0,
                // 0);
                // PIDController yController = new PIDController(AutoConstants.kPYController, 0,
                // 0);
                // ProfiledPIDController thetaController = new ProfiledPIDController(
                // AutoConstants.kPThetaController, 0, 0,
                // AutoConstants.kThetaControllerConstraints);
                // thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // // 4. Construct command to follow trajectory
                // SwerveControllerCommand swerveControllerCommand = new
                // SwerveControllerCommand(
                // trajectory,
                // swerveSubsystem::getPose,
                // DriveConstants.kDriveKinematics,
                // xController,
                // yController,
                // thetaController,
                // swerveSubsystem::setModuleStates,
                // swerveSubsystem);

                // // 5. Add some init and wrap-up, and return everything
                // return new SequentialCommandGroup(
                // new InstantCommand(() ->
                // swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                // swerveControllerCommand,
                // new InstantCommand(() -> swerveSubsystem.stopModules()));
        }
}
