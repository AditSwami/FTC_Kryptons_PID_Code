package org.firstinspires.ftc.teamcode.FTC_Code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp (name = "Drive")



public class Drive extends OpMode {
    Driver_settings driver = new Driver_settings();

    @Override
    public void init() {
        driver.init(hardwareMap);

    }

    @Override
    public void loop() {
        driver.moveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
