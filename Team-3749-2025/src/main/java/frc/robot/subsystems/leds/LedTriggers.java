package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.arm.climb.ClimbArmConstants.ArmStates;
import frc.robot.subsystems.leds.LEDConstants.LEDColor;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public class LedTriggers {

    public static void createLEDTriggers() {
        
        Trigger coralHasPieceTrigger = new Trigger(() -> Robot.coralRoller.hasPiece());
        coralHasPieceTrigger.onTrue(Commands.runOnce(() -> Robot.led.setLEDColor(LEDColor.CORAL_ARM_HAS_PIECE)));
        

        Trigger OTFTrigger = new Trigger(() -> Robot.swerve.getIsOTF());
        OTFTrigger.onTrue(Commands.runOnce(() -> Robot.led.setLEDColor(LEDColor.RAINBOW)));
        OTFTrigger.onFalse(Commands.runOnce(() -> Robot.led.setLEDColor(Robot.led.getTeamColorLED())));

        Trigger climbTrigger = new Trigger(() -> Robot.climbArm.getState().equals(ArmStates.CLIMB));
        climbTrigger.onTrue(Commands.runOnce(() ->Robot.led.setLEDColor(LEDColor.CLIMB)));

    }

}
