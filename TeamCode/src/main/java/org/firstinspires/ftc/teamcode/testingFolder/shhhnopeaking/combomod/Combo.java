package org.firstinspires.ftc.teamcode.testingFolder.shhhnopeaking.combomod;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.ArrayList;

public class Combo {

    ArrayList<GamepadKeys.Button> buttons = new ArrayList();

    public Combo(GamepadKeys.Button... buttons){
        for(GamepadKeys.Button i : buttons){
            this.buttons.add(i);
        }
    }

    public void setPushTypes(){

    }
}
