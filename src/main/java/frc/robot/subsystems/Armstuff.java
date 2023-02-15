// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Armstuff extends SubsystemBase {
  /** Creates a new Armstuff. */
  //left and right arm stuff 
  private final VictorSP m_TiltArm =  new VictorSP(Constants.ArmConstants.kTilt);
  Encoder tiltEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
  // up and down arm stuff
  private final VictorSP m_ExstendedArm = new VictorSP(Constants.ArmConstants.kExstend);
  DigitalInput armswitch = null;
  Encoder extendEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
  public Armstuff() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
