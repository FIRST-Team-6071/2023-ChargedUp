// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends PIDCommand {
  DriveSubsystem m_Drive;
  Timer timer = new Timer();

  /** Creates a new AutoBalance. */
  public AutoBalance(DriveSubsystem p_Drive) {
    super(
        // The controller that the command will use
        new PIDController(8, 2, 0),
        // This should return the measurement
        () -> p_Drive.getPitchValue(),
        // This should return the setpoint (can also be a constant)
        () -> -3,
        // This uses the output
        output -> {
          // Use the output here
          p_Drive.drive(MathUtil.clamp(output, -0.09, 0.09), 0, 0, false, false);
        });

        m_Drive = p_Drive;

        addRequirements(p_Drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!(m_Drive.getPitchValue() > (-3 - .50) && m_Drive.getPitchValue() < (-3 + .50))) timer.reset();

    if (timer.get() > 2) {
      return true;
    } else {
      return false;
    }

    // return false;
    // return m_Drive.getPitchValue() > (-3 - .50) && m_Drive.getPitchValue() < (-3 + .50);
  }
}
