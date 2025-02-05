package frc.robot.subsystems.roller.real;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.roller.PhotoelectricIO;

public class JTVisiSight implements PhotoelectricIO{

    private DigitalInput photoelectricSensor = new DigitalInput(5);

    public JTVisiSight(){

    }

    @Override
    public void updateData(PhotoelectricData data){
        data.sensing = photoelectricSensor.get();
    }
}