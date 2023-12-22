// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
  CANSparkMax mLiftMotor = new CANSparkMax(Constants.ArmConstants.kLiftMotorPort, MotorType.kBrushless);
  SparkMaxPIDController liftPidController;
  RelativeEncoder liftEncoder;
  // Supplier<Double> setpoint = () -> 0.0;
  double setpoint = 0;
  double adjustment = 0.0;

  /** Creates a new Lift. */
  public Lift() {
    // mLiftMotor.restoreFactoryDefaults();
    liftEncoder = mLiftMotor.getEncoder();
    liftEncoder.setPositionConversionFactor(Constants.ArmConstants.kLiftRot2Rad);
    liftPidController = mLiftMotor.getPIDController();
    liftPidController.setP(Constants.ArmConstants.kPLift);
    liftPidController.setI(0);
    liftPidController.setD(0);
    liftPidController.setOutputRange(-.25, .25);
    mLiftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    mLiftMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.ArmConstants.kLiftForwardLimit);
    mLiftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    mLiftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.ArmConstants.kLiftReverseLimit);
    resetEncoders();
  }

  // public void Raise(Supplier<Double> position) {
  //   setpoint = position;
  //   liftPidController.setReference(setpoint.get(), ControlType.kPosition);
  //   SmartDashboard.putNumber("lift angle", liftEncoder.getPosition());
  //   SmartDashboard.putNumber("lift setpoint", setpoint.get());
  // }

  public CommandBase setSetpoint(Supplier<Double> position) {
    return runOnce(() -> setpoint = position.get());
  }

  // public CommandBase moveToSetpoint() {
  //   System.out.println("aaaa");
  //   return run(() -> liftPidController.setReference(setpoint, ControlType.kPosition));
  // }

  public CommandBase resetEncoders() {
    return runOnce(() -> liftEncoder.setPosition(0));
  }

  public boolean atSetpoint() {
    return getPosition() == setpoint;
  }

  public double getPosition() {
    return liftEncoder.getPosition();
  }

  // public CommandBase move(Supplier<Double> position) {
  //   SmartDashboard.putNumber("lift setpoint", position.get());
  //   return run(() -> liftPidController.setReference(position.get(), ControlType.kPosition));
  // }

  public CommandBase adjustUp(Supplier<Double> amount) {
    // adjustment = amount.get();
    // adjustment = Math.abs(adjustment) > .12 ? adjustment : 0.0;
    SmartDashboard.putNumber("lift nudge", adjustment);
    return runOnce(() -> setpoint = setpoint + amount.get());
    // return run(() -> liftPidController.setReference(getPosition() + adjustment, ControlType.kPosition));
  }

  public CommandBase adjustDown(Supplier<Double> amount) {
    // adjustment = amount.get();
    // adjustment = Math.abs(adjustment) > .12 ? adjustment : 0.0;
    SmartDashboard.putNumber("lift nudge", adjustment * -1);
    return runOnce(() -> setpoint = setpoint + amount.get());
    // return run(() -> liftPidController.setReference(getPosition() - adjustment, ControlType.kPosition));
  }

  public CommandBase adjustSetpoint(Supplier<Double> amount) {
    SmartDashboard.putNumber("Lift Adjustment", amount.get());
    return run(() -> setpoint = setpoint + amount.get());
  }

  public void adjust(Supplier<Double> amount) {
    adjustment = amount.get();
    adjustment = Math.abs(adjustment) > .15 ? adjustment : 0.0;
    adjustment = adjustment * .2;
    SmartDashboard.putNumber("lift adjustment", adjustment);
    setpoint = setpoint + adjustment;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("lift angle", liftEncoder.getPosition());
    SmartDashboard.putNumber("lift setpoint", setpoint);
    // SmartDashboard.putNumber("lift setpoint", setpoint.get());
    SmartDashboard.putBoolean("lift at setpoint", atSetpoint());
    SmartDashboard.putData(this);
    liftPidController.setReference(setpoint, ControlType.kPosition);
  }
}
