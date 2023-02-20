// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new Armstuff. */
  //left and right arm stuff 
  private final VictorSPX m_TiltArm =  new VictorSPX(Constants.ArmConstants.kTilt);
 Encoder tiltEncoder = new Encoder(Constants.MaxETilt.KTMax, Constants.MinETilt.KTMin);
  // up and down arm stuff
  private final VictorSPX m_ExtendedArm = new VictorSPX(Constants.ArmConstants.kExtend);
  DigitalInput armswitch = null;
  Encoder extendEncoder = new Encoder(Constants.MaxExtend.KEMax, Constants.MinRetract.KEMin);
  public ArmSubsystem() {
    
  }
  public void up (){
    m_ExtendedArm.set(Constants.ArmExtendSpeed.ExtendSpeed);
  }
  public void down(){
    m_ExtendedArm.set(-Constants.ArmExtendSpeed.ExtendSpeed);
  }
  public void extend(){
  m_TiltArm.set(Constants.ArmDirectionSpeed.DSpeed);
  }
  public void retract(){
  m_TiltArm.set(Constants.ArmDirectionSpeed.DSpeed);
  }

  public void TiltEncoder(int KTMax, int KTMin){
if (KTMax > 1000){
  tiltEncoder.getStopped();
  }

if (KTMin == 0){
  // whatever encoder go is in java
}
}

public void extendEncoder(int KEMax, int KEMin){
  if (KEMax > 1000){
    extendEncoder.getStopped();
  }

  if (KEMin == 0){
    // whatever encoder go is in java
  }
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
