package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.67;
        public static final double kTurningMotorGearRatio = 48 / 40;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderPulse2Meter = kDriveEncoderRot2Meter / 80;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kTurningENcoderPulse2Rad = kTurningEncoderRot2Rad / 415.1;
        public static final double kAbsoluteEncoderRot2Rad = 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 1;
        public static final double kPAbsoluteTurning = .6;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(18);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(26);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 7;

        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kBackRightTurningMotorPort = 8;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int[] kFrontLeftTurningEncoderPorts = { 0, 1 };
        public static final int[] kBackLeftTurningEncoderPort = { 2, 3 };
        public static final int[] kFrontRightTurningEncoderPort = { 4, 5 };
        public static final int[] kBackRightTurningEncoderPort = { 7, 8 };

        public static final int kFrontLeftAbsoluteEncoderPort = 0;
        public static final int kBackLeftAbsoluteEncoderPort = 1;
        public static final int kFrontRightAbsoluteEncoderPort = 2;
        public static final int kBackRightAbsoluteEncoderPort = 3;

        public static final int[] kFrontLeftDriveEncoderPorts = { 10, 11 };
        public static final int[] kBackLeftDriveEncoderPorts = { 18, 19 };
        public static final int[] kFrontRightDriveEncoderPorts = { 6, 9 };
        public static final int[] kBackRightDriveEncoderPorts = { 12, 13 };

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        public static final double kFrontLeftDriveAbsoluteEncoderOffset = -1.46; // Rotation2d.fromDegrees(234).getRadians();
        public static final double kBackLeftDriveAbsoluteEncoderOffset = -.97; // Rotation2d.fromDegrees(138).getRadians();
        public static final double kFrontRightDriveAbsoluteEncoderOffset = .21; // Rotation2d.fromDegrees(350).getRadians();
        public static final double kBackRightDriveAbsoluteEncoderOffset = -.07; // Rotation2d.fromDegrees(213).getRadians();

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.25;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond; // /4
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class ArmConstants {
        public static final int kLiftMotorPort = 10;
        public static final double kPLift = .4;
        public static final double kLiftGearRatio = 1 / 100 * 22 / 60;
        public static final double kLiftRot2Rad = 360 * kLiftGearRatio;
        public static final double kLiftReverseLimit = 2f;
        public static final double kLiftForwardLimit = 65f;

        public static final int kExtendMotorPort = 11;
        public static final double kPExtend = .05;
        public static final double kExtendDiameter = .1; // meters
        public static final double kExtendGearRatio = 1 / 4;
        public static final double kExtendRot2Meter = kExtendGearRatio * Math.PI * kExtendDiameter;
        public static final double kExtendReverseLimit = 0.0f;
        public static final double kExtendForwardLimit = 6.9f;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.15;
    }
}
