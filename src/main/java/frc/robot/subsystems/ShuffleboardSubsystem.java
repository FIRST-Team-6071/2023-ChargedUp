 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShuffleboardSubsystem extends SubsystemBase {
  ArmSubsystem m_Arm;
  DriveSubsystem m_Drive;
  ArmExtensionSubsystem m_ArmExt;
  SendableChooser<String> m_autoChooser = new SendableChooser<>();


  /** Creates a new ShuffleboardSubsystem. */
  public ShuffleboardSubsystem(ArmSubsystem p_ArmSubsystem, DriveSubsystem p_DriveSubsystem, ArmExtensionSubsystem p_ArmExtensionSubsystem) {
    m_Arm = p_ArmSubsystem;
    m_Drive = p_DriveSubsystem;
    m_ArmExt = p_ArmExtensionSubsystem;

    m_autoChooser.setDefaultOption("Center", "center");
    m_autoChooser.addOption("Left/Right", "side");
    m_autoChooser.addOption("Arm Test", "arm");
    // SmartDashboard.putNumber("Wanted Arm Tilt Position", m_Arm.GetWantedArmTiltPosition());
    SmartDashboard.putData(m_autoChooser);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Extension Switch Left", m_ArmExt.extensionSwitchLeft.get());
    SmartDashboard.putBoolean("Extension Switch Right", m_ArmExt.extensionSwitchRight.get());
    SmartDashboard.putBoolean("Has Extension Zeroed", m_ArmExt.GetHasExtensionZeroed());
    SmartDashboard.putBoolean("Arm Tilt Encoder Connected", m_Arm.GetTiltEncoderConnected());
    SmartDashboard.putNumber("Arm Position", m_ArmExt.GetArmExtensionEncoderValue());
    SmartDashboard.putNumber("Arm Position Percent", (m_ArmExt.GetArmExtensionEncoderValue() % Constants.Arm.Extension.Encoder.k_Max));
    SmartDashboard.putNumber("Arm Position Slash", (m_ArmExt.GetArmExtensionEncoderValue() / Constants.Arm.Extension.Encoder.k_Max));
    SmartDashboard.putNumber("Arm Tilt Position", m_Arm.GetTiltEncoderValue());
    SmartDashboard.putNumber("Raw Arm Tilt Position", m_Arm.GetRawTiltEncoderPosition());
    SmartDashboard.putNumber("Wanted Arm Tilt Position", m_Arm.GetWantedArmTiltPosition());
    
    SmartDashboard.putNumber("Pose X", m_Drive.m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Y", m_Drive.m_odometry.getPoseMeters().getY());
    
    SmartDashboard.putNumber("Angle Value", m_Drive.getAngleValue());
    SmartDashboard.putNumber("Pitch Value", m_Drive.getPitchValue());
    SmartDashboard.putNumber("Roll Value", m_Drive.getRollValue());
    
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

  public String getChosenAuton() {
    return "side";
    // return m_autoChooser.getSelected();
  }
}
