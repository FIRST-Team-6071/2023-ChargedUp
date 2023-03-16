// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShuffleboardSubsystem extends SubsystemBase {
  ArmSubsystem m_Arm;
  DriveSubsystem m_Drive;


  /** Creates a new ShuffleboardSubsystem. */
  public ShuffleboardSubsystem(ArmSubsystem p_ArmSubsystem, DriveSubsystem p_DriveSubsystem) {
    m_Arm = p_ArmSubsystem;
    m_Drive = p_DriveSubsystem;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Extension Switch Left", m_Arm.extensionSwitchLeft.get());
    SmartDashboard.putBoolean("Extension Switch Right", m_Arm.extensionSwitchRight.get());
    SmartDashboard.putBoolean("Has Extension Zeroed", m_Arm.GetHasExtensionZeroed());
    SmartDashboard.putBoolean("Arm Tilt Encoder Connected", m_Arm.GetTiltEncoderConnected());
    SmartDashboard.putNumber("Arm Position", m_Arm.GetArmExtensionEncoderValue());
    SmartDashboard.putNumber("Arm Position Percent", (m_Arm.GetArmExtensionEncoderValue() % Constants.Arm.Extension.Encoder.k_Max));
    SmartDashboard.putNumber("Arm Position Slash", (m_Arm.GetArmExtensionEncoderValue() / Constants.Arm.Extension.Encoder.k_Max));
    SmartDashboard.putNumber("Arm Tilt Position", m_Arm.GetTiltEncoderValue());
    SmartDashboard.putNumber("Wanted Arm Tilt Position", m_Arm.GetWantedArmTiltPosition());

    switch (m_Drive.GetDriveMode()) {
      case 1:
        SmartDashboard.putString("Drive Mode", "Fast");
        break;
    
      case 2:
        SmartDashboard.putString("Drive Mode", "Slow");
        break;
    
      default:
        SmartDashboard.putString("Drive Mode", "Normal");
        break;
    }
    // % Constants.Arm.Extension.Encoder.k_Max
  }
}
