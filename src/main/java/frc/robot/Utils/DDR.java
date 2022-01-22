package frc.robot.Utils;

import edu.wpi.first.wpilibj.GenericHID;

public class DDR extends GenericHID {
    
    public enum Button {
        kLeft(0),
        kUp(2),
        kRight(3),
        kDown(1),
        kX(6),
        kO(7),
        kSq(5),
        kTr(4),
        kSelect(8),
        kStart(9);

        public final int value;

        Button(int value) {
            this.value = value;
        }

    }

    public DDR(int port) {
        super(port);
    }


    /** Get button inputs **/
    public boolean getTopLeft() {
        return getRawButton(Button.kX.value);
    }
    public boolean getTopLeftPressed() {
        return getRawButtonPressed(Button.kX.value);
    }
    public boolean getTopLeftReleased() {
        return getRawButtonReleased(Button.kX.value);
    }


    public boolean getTopRight() {
        return getRawButton(Button.kO.value);
    }
    public boolean getTopRightPressed() {
        return getRawButtonPressed(Button.kO.value);
    }
    public boolean getTopRightReleased() {
        return getRawButtonReleased(Button.kO.value);
    }


    public boolean getBottomRight() {
        return getRawButton(Button.kSq.value);
    }
    public boolean getBottomRightPressed() {
        return getRawButton(Button.kSq.value);
    }
    public boolean getBottomRightReleased() {
        return getRawButton(Button.kSq.value);
    }


    public boolean getBottomLeft() {
        return getRawButton(Button.kTr.value);
    }
    public boolean getBottomLeftPressed() {
        return getRawButtonPressed(Button.kTr.value);
    }
    public boolean getBottomLeftReleased() {
        return getRawButtonReleased(Button.kTr.value);
    }


    public boolean getUp() {
        return getRawButton(Button.kUp.value);
    }
    public boolean getUpPressed() {
        return getRawButtonPressed(Button.kUp.value);
    }
    public boolean getUpReleased() {
        return getRawButtonReleased(Button.kUp.value);
    }


    public boolean getRight() {
        return getRawButton(Button.kRight.value);
    }
    public boolean getRightPressed() {
        return getRawButtonPressed(Button.kRight.value);
    }
    public boolean getRightReleased() {
        return getRawButtonReleased(Button.kRight.value);
    }


    public boolean getDown() {
        return getRawButton(Button.kDown.value);
    }
    public boolean getDownPressed() {
        return getRawButtonPressed(Button.kDown.value);
    }
    public boolean getDownReleased() {
        return getRawButtonReleased(Button.kDown.value);
    }


    public boolean getLeft() {
        return getRawButton(Button.kLeft.value);
    }
    public boolean getLeftPressed() {
        return getRawButtonPressed(Button.kLeft.value);
    }
    public boolean getLeftReleased() {
        return getRawButtonReleased(Button.kLeft.value);
    }


    public boolean getStart() {
        return getRawButton(Button.kStart.value);
    }
    public boolean getStartPressed() {
        return getRawButtonPressed(Button.kStart.value);
    }
    public boolean getStartReleased() {
        return getRawButtonReleased(Button.kStart.value);
    }


    public boolean getSelect() {
        return getRawButton(Button.kSelect.value);
    }
    public boolean getSelectPressed() {
        return getRawButtonPressed(Button.kSelect.value);
    }
    public boolean getSelectReleased() {
        return getRawButtonReleased(Button.kSelect.value);
    }


}

