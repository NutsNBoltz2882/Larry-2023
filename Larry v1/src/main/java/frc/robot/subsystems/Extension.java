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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Extension extends SubsystemBase {
  CANSparkMax mExtendMotor = new CANSparkMax(Constants.ArmConstants.kExtendMotorPort, MotorType.kBrushless);
  SparkMaxPIDController extendPidController;
  RelativeEncoder extendEncoder;
  double setpoint = 0.0;
  double adjustment = 0;

  /** Creates a new Lift. */
  public Extension() {
    // mExtendMotor.restoreFactoryDefaults();
    mExtendMotor.setInverted(true);
    extendEncoder = mExtendMotor.getEncoder();
    extendEncoder.setPositionConversionFactor(Constants.ArmConstants.kExtendRot2Meter);
    extendPidController = mExtendMotor.getPIDController();
    extendPidController.setP(Constants.ArmConstants.kPExtend);
    extendPidController.setI(0);
    extendPidController.setD(0);
    extendPidController.setOutputRange(-1, 1);
    mExtendMotor.enableSoftLimit(SoftLimitDirection.kForward, true);    
    mExtendMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.ArmConstants.kExtendForwardLimit);
    mExtendMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    mExtendMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.ArmConstants.kExtendReverseLimit);
    resetEncoders();
  }

  // public void Extend(Supplier<Double> position) {
  //   setpoint = position;
  //   extendPidController.setReference(position.get(), ControlType.kPosition);
  //   SmartDashboard.putNumber("extend angle", extendEncoder.getPosition());
  //   SmartDashboard.putNumber("extend setpoint", setpoint.get());
  // }

  public CommandBase setSetpoint(Supplier<Double> position) {
    // double temp = position.get() < 0  ? adjustment : 0.0;
    return runOnce(() -> setpoint = position.get());
  }

  // public CommandBase moveToSetpoint() {
  //   return run(() -> extendPidController.setReference(setpoint, ControlType.kPosition));
  // }

  public CommandBase resetEncoders() {
    return runOnce(() -> extendEncoder.setPosition(0));
  }

  public boolean atSetpoint() {
    return getPosition() == setpoint;
  }

  public double getPosition() {
    return extendEncoder.getPosition();
  }

  public CommandBase setSpeed(Supplier<Double> speed) {
    // SmartDashboard.putNumber("extend speed", speed.get());
    return runEnd(() -> mExtendMotor.set(speed.get()), () -> mExtendMotor.stopMotor());
  }

  public CommandBase move(Supplier<Double> position) {
    SmartDashboard.putNumber("extend setpoint", position.get());
    return run(() -> extendPidController.setReference(position.get(), ControlType.kPosition));
  }

  // public CommandBase adjust(Supplier<Double> amount) {
  //   SmartDashboard.putNumber("extend nudge", amount.get());
  //   return run(() -> extendPidController.setReference(getPosition() + amount.get(), ControlType.kPosition));
  // }

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
    if(setpoint < 0) setpoint = 0;
    if(setpoint > 8) setpoint = 8;
    SmartDashboard.putNumber("extend angle", extendEncoder.getPosition());
    SmartDashboard.putNumber("extend speed", mExtendMotor.get());
    SmartDashboard.putNumber("extend setpoint", setpoint);
    // SmartDashboard.putNumber("extend setpoint", setpoint.get());
    SmartDashboard.putBoolean("extend at setpoint", atSetpoint());
    SmartDashboard.putData(this);
    extendPidController.setReference(setpoint, ControlType.kPosition);
  }
}