package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.buttons.JoystickIO;
import frc.robot.subsystems.leds.LEDConstants.LEDColor;

public class LedTriggers {

    public static void createLEDTriggers() {

        Trigger coralHasPieceTrigger = new Trigger(() -> Robot.coralRoller.hasPiece());
            coralHasPieceTrigger.onTrue(Commands.runOnce(() -> Robot.led.setColor(LEDColor.RAINBOW)));
            coralHasPieceTrigger.onFalse(Commands.runOnce(() -> Robot.led.setColor(Robot.led.getTeamColorLED())));

        Trigger scoringModeTrigger = new Trigger(() -> JoystickIO.getButtonBoard().getScoringMode() == JoystickIO
                .getButtonBoard().getPreviousScoringMode());

        scoringModeTrigger.onChange(Commands.runOnce(() -> {
            switch (JoystickIO.getButtonBoard().getScoringMode()) {
                case ALGAE:
                    Robot.led.setColor(LEDColor.ALGAE);
                    break;
                case L1:
                    Robot.led.setColor(LEDColor.L1);

                    break;
                case L2:
                    Robot.led.setColor(LEDColor.L2);

                    break;
                case L3:
                    Robot.led.setColor(LEDColor.L3);

                    break;
                case L4:
                    Robot.led.setColor(LEDColor.L4);
                    break;
                default:
                    Robot.led.setColor(Robot.led.getTeamColorLED());
            }

        }));
    }

}
