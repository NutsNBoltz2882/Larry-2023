// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  DoubleSolenoid grabSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  /** Creates a new Grabber. */
  public Grabber() {
    // compressor.disable();
    grabSolenoid.set(Value.kForward);
  }

  public CommandBase grabCommand() {
    // SmartDashboard.putBoolean("extend", true);
    return runOnce(() -> grabSolenoid.toggle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(this);
  }
}
