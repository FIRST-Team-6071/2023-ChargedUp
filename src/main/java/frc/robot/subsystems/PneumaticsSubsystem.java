// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
  Compressor phCompressor = new Compressor(
    Constants.PneumaticsConstants.PNEUMATICS_MODULE, 
    PneumaticsModuleType.REVPH
  );

  DoubleSolenoid clawSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    Constants.PneumaticsConstants.Claw.EXTENSION, 
    Constants.PneumaticsConstants.Claw.RETRACTION
  );

  /** Creates a new Pneumatics. */
  public PneumaticsSubsystem() {
    phCompressor.enableDigital();
    clawSolenoid.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(phCompressor.getPressureSwitchValue());
  }

  public boolean isAtPressure() {
    return phCompressor.getPressureSwitchValue();
  }

  public CommandBase openClaw() {
    return runOnce(
      () -> {
        clawSolenoid.set(Value.kForward);
      }
    );
  }

  public CommandBase closeClaw() {
    return runOnce(
      () -> {
        clawSolenoid.set(Value.kReverse);
      }
    );
  }
}
