package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.revrobotics.CANEncoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final VictorSPX driveMotor;
    private final VictorSPX turningMotor;

    private final Encoder driveEncoder;
    private final Encoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogEncoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;

    private double turnOutput;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
        int[] driveEncoderIDs, int[] turnEncoderIDs, int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogEncoder(absoluteEncoderID);

        driveMotor = new VictorSPX(driveMotorId);
        turningMotor = new VictorSPX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = new Encoder(driveEncoderIDs[0], driveEncoderIDs[1]);
        turningEncoder = new Encoder(turnEncoderIDs[0], turnEncoderIDs[1]);

        // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderPulse2Meter);
        // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setDistancePerPulse(ModuleConstants.kTurningENcoderPulse2Rad);
        // turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        absoluteEncoder.setDistancePerRotation(ModuleConstants.kAbsoluteEncoderRot2Rad);
        // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        // SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getChannel() + "] drive position", driveEncoder.getDistance());
        return driveEncoder.getDistance();
    }

    public double getTurningPosition() {
        return turningEncoder.getDistance();
    }

    public double getDriveVelocity() {
        return driveEncoder.getRate();
    }

    public double getTurningVelocity() {
        return turningEncoder.getRate();
    }

    public double getAbsoluteEncoderRad() {
        // double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        // angle *= 2.0 * Math.PI;
        // angle -= absoluteEncoderOffsetRad;
            // double angle = absoluteEncoder.getAbsolutePosition() * 2 * Math.PI - absoluteEncoderOffset;
            // return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition()).getRadians();
        // return (absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffset)* absoluteEncoder.getDistancePerRotation();
    }

    public void resetEncoders() {
        // return runOnce(
        //     () -> driveEncoder.reset()
        //     .alongWith(() -> absoluteEncoder.reset())
        //     );
        driveEncoder.reset();
        absoluteEncoder.reset();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
        // return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad() - absoluteEncoderOffset));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
        // return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.1) { // makes it not move if speed low
            stop();
            return;
        }
        // SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] original state", state.toString());
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turnOutput = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        // SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getChannel() + "] turn output", turnOutput);
        turningMotor.set(ControlMode.PercentOutput, turnOutput);
        // SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", getState().toString());
        // SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] desired state", state.toString());
        // double turnOutput = turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians());
        // double turnOutput = turningPidController.calculate(absoluteEncoder.getAbsolutePosition(), state.angle.getRadians());
    }

    public boolean absReset() {
        turningPidController.setTolerance(Math.PI / 60);
        turnOutput = turningPidController.calculate(getAbsoluteEncoderRad(), absoluteEncoderOffset);
        turningMotor.set(ControlMode.PercentOutput, turnOutput);
        driveMotor.set(ControlMode.PercentOutput, 0);
        return turningPidController.atSetpoint();
    }

    // public PIDCommand absReset() {
    //     return new PIDCommand(turningPidController, getAbsoluteEncoderRad(), 0, 
    //         output -> turningMotor.set(ControlMode.PercentOutput, output), this);
    // }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }

    public void values(String module) {
        // SmartDashboard.putNumber("Mod " + module + " angle", getTurningPosition());
        // SmartDashboard.putNumber("Mod " + module + " velocity", getDriveVelocity());
        // SmartDashboard.putNumber("Mod " + module + " angle", getTurningPosition());
    }
}
