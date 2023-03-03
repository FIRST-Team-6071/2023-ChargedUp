// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShuffleboardSubsystem extends SubsystemBase {
  ArmSubsystem m_Arm;


  /** Creates a new ShuffleboardSubsystem. */
  public ShuffleboardSubsystem(ArmSubsystem p_ArmSubsystem) {
    m_Arm = p_ArmSubsystem;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Extension Switch Left", m_Arm.extensionSwitchLeft.get());
    SmartDashboard.putBoolean("Extension Switch Right", m_Arm.extensionSwitchRight.get());
    SmartDashboard.putBoolean("Has Extension Zeroed", m_Arm.GetHasExtensionZeroed());
    SmartDashboard.putNumber("Arm Position", m_Arm.GetArmExtensionEncoderValue());
    SmartDashboard.putNumber("Arm Position Percent", (m_Arm.GetArmExtensionEncoderValue() % Constants.Arm.Extension.Encoder.k_Max));
    SmartDashboard.putNumber("Arm Position Slash", (m_Arm.GetArmExtensionEncoderValue() / Constants.Arm.Extension.Encoder.k_Max));

    // % Constants.Arm.Extension.Encoder.k_Max
  }
}
