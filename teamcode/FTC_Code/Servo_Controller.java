package org.firstinspires.ftc.teamcode.FTC_Code;

import android.os.Debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp(name = "Servo_Master")
public class Servo_Controller extends OpMode {
    public CRServo servo;
    @Override
    public void init() {
        Debug.waitForDebugger();
        servo = hardwareMap.get(CRServo.class, "servo");
    }

    @Override
    public void loop(){
        if(gamepad1.a){
            servo.setPower(1);
        }
        else if (gamepad1.b){
            servo.setPower(-1);
        }
        servo.setPower(0);
    }
}