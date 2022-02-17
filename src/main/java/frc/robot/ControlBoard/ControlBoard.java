package frc.robot.ControlBoard;

//totally not copying team 254's idea
//Code is here but probably won't implemenet it into the rest of the code for awhile
public class ControlBoard {
    
    private static ControlBoard _instance = null;
    private final IDriveBoard dBoard;

    public static ControlBoard getInstance() {
        if (_instance == null) {
            _instance = new ControlBoard();
        }

        return _instance;
    }

    private ControlBoard() {
        dBoard = JoystickDrive.getInstance();
    }

    public double getThrottle() {
        return dBoard.getThrottle();
    }

    public double getRight() {
        return dBoard.getRight();
    }

    public double getRotation() {
        return dBoard.getRotation();
    }

}
