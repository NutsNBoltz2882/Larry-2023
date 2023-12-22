// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor extends SubsystemBase {
  private final ColorSensorV3 colorSensor = new ColorSensorV3(Port.kMXP);
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color cube = new Color(152, 262, 176);
  private final Color cone = new Color(240, 376, 139);
  /** Creates a new ColorSensor. */
  public ColorSensor() {
    colorMatcher.addColorMatch(cone);
    colorMatcher.addColorMatch(cube);
  }

  public boolean pieceIn() {
    SmartDashboard.putNumber("R Value",  colorSensor.getRed());
    SmartDashboard.putNumber("G Value",  colorSensor.getGreen());
    SmartDashboard.putNumber("B Value",  colorSensor.getBlue());
    SmartDashboard.putNumber("Grabber Proximity",  colorSensor.getProximity());
    ColorMatchResult match = colorMatcher.matchClosestColor(colorSensor.getColor());
    return match.color == cone || match.color == cube;
  }

  public void detect() {
    SmartDashboard.putNumber("R Value",  colorSensor.getRed());
    SmartDashboard.putNumber("G Value",  colorSensor.getGreen());
    SmartDashboard.putNumber("B Value",  colorSensor.getBlue());
    SmartDashboard.putNumber("Grabber Proximity",  colorSensor.getProximity());
    SmartDashboard.putBoolean("piece", pieceIn());
  }
  
  public CommandBase output() {
    return run(() -> System.out.println(colorSensor.getColor()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("R Value",  colorSensor.getRed());
    // SmartDashboard.putNumber("G Value",  colorSensor.getGreen());
    // SmartDashboard.putNumber("B Value",  colorSensor.getBlue());
    // SmartDashboard.putNumber("Grabber Proximity",  colorSensor.getProximity());
  }
}
