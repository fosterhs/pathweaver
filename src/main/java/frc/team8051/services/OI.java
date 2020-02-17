/*
Code for controller
 */
package frc.team8051.services;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;


public class OI {
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_LB = 5;
    public static final int BUTTON_RB = 6;
    public static final int BUTTON_BACK = 7;
    public static final int BUTTON_START = 8;

    //declaring button variables 1-8
    private JoystickButton buttonA, buttonB, buttonX, buttonY, buttonLB, buttonRB, buttonBACK, buttonSTART;
    private Joystick joystick;

    public OI() {
        joystick = new Joystick(0);
        //setting button variables to numbers on controller
        buttonA = new JoystickButton(joystick, BUTTON_A);
        buttonB = new JoystickButton(joystick, BUTTON_B);
        buttonX = new JoystickButton(joystick, BUTTON_X);
        buttonY = new JoystickButton(joystick, BUTTON_Y);
        buttonLB = new JoystickButton(joystick, BUTTON_LB);
        buttonRB = new JoystickButton(joystick, BUTTON_RB);
        buttonBACK = new JoystickButton(joystick, BUTTON_BACK);
        buttonSTART = new JoystickButton(joystick, BUTTON_START);
    }

    public void initializeBind() {
        
    }
    public double getRightXAxis() {
         return joystick.getRawAxis(4); 
    }

    public double getRightYAxis() {
        return joystick.getRawAxis(5);
    }

    public double getLeftXAxis() {
        return joystick.getRawAxis(0);
    }

    public double getLeftYAxis() {
        return joystick.getRawAxis(1);
    }

}
