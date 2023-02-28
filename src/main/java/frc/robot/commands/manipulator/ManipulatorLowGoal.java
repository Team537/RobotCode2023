// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arminout.ArmInOutHighGoal;
import frc.robot.commands.arminout.ArmInOutLowGoal;
import frc.robot.commands.arminout.ArmInOutMidGoal;
import frc.robot.commands.armpivot.ArmPivotHighGoal;
import frc.robot.commands.armpivot.ArmPivotLowGoal;
import frc.robot.commands.armpivot.ArmPivotMidGoal;
import frc.robot.commands.led.LedHighGoal;
import frc.robot.commands.led.LedLowGoal;
import frc.robot.commands.led.LedMidGoal;
import frc.robot.commands.wrist.WristHighGoal;
import frc.robot.commands.wrist.WristLowGoal;
import frc.robot.commands.wrist.WristMidGoal;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.manipulator.ArmInOut;
import frc.robot.subsystems.manipulator.ArmPivot;
import frc.robot.subsystems.manipulator.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManipulatorLowGoal extends SequentialCommandGroup {


  /** Creates a new ManipulatorHighGoal. */
  public ManipulatorLowGoal(ArmPivot m_ArmPivot, ArmInOut m_ArmInOut, Wrist m_Wrist, LED m_LED) {

    
     
   
    
     
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( new ParallelCommandGroup(new LedLowGoal(m_LED),
    new ParallelCommandGroup(new ArmPivotLowGoal(m_ArmPivot),  new WristLowGoal(m_Wrist)), new WaitCommand(1), new ArmInOutLowGoal(m_ArmInOut)));
  }
}
