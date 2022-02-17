package frc.robot.ControlBoard;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.IOConstants;

public class JoystickDrive implements IDriveBoard {
    
    private final Joystick _j1;
    private final Joystick _j2;
    private static JoystickDrive _instance = null;

    public static JoystickDrive getInstance() {
        if(_instance == null) {
            _instance = new JoystickDrive();
        }

        return _instance;
    }

    private JoystickDrive() {
        _j1 = new Joystick(IOConstants.LEFT_JOYSTICK);
        _j2 = new Joystick(IOConstants.RIGHT_JOYSTICK);
    }

    @Override
    public double getThrottle() {
        return _j1.getY();
    }

    @Override
    public double getRight() {
        return _j1.getX();
    }

    @Override
    public double getRotation() {
        return _j2.getX();
    }


}
