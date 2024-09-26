package org.firstinspires.ftc.teamcode.FTC_Code;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Driver_settings {

    public DcMotor topLeftDriveMotor;
    public DcMotor bottomLeftDriveMotor;
    public DcMotor topRightDriveMotor;
    public DcMotor bottomRightDriveMotor;

    // Initialization of hardware map
    public void init(HardwareMap hwMap) {

        // Initialize the motors from the hardware map
        topLeftDriveMotor = hwMap.get(DcMotor.class, "motor_topLeft");
        bottomLeftDriveMotor = hwMap.get(DcMotor.class, "motor_botLeft");
        topRightDriveMotor = hwMap.get(DcMotor.class, "motor_topRight");
        bottomRightDriveMotor = hwMap.get(DcMotor.class, "motor_botRight");

        // Set motors to run without encoders
        topLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse direction for the left side motors
        topLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        topRightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        bottomRightDriveMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to brake when power is set to zero
        topLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize motor power to zero
        stopMotors();
    }

    // Method to stop all motors
    public void stopMotors() {
        topLeftDriveMotor.setPower(0);
        bottomLeftDriveMotor.setPower(0);
        topRightDriveMotor.setPower(0);
        bottomRightDriveMotor.setPower(0);
    }

    // Set equal power to all wheels (for forward/backward movement)
    public void power(double output) {
        topLeftDriveMotor.setPower(-output);
        bottomLeftDriveMotor.setPower(-output);
        topRightDriveMotor.setPower(output);
        bottomRightDriveMotor.setPower(output);
    }

    // Method to move robot using joystick inputs for mecanum drive
    public void moveRobot(double leftStickY, double leftStickX, double rightStickX) {
        // Calculate motor power based on joystick inputs
        double topLeftPower = leftStickY + leftStickX + rightStickX;
        double bottomLeftPower = leftStickY - leftStickX + rightStickX;
        double topRightPower = leftStickY - leftStickX - rightStickX;
        double bottomRightPower = leftStickY + leftStickX - rightStickX;

        // Set motor power
        topLeftDriveMotor.setPower(topLeftPower);
        topRightDriveMotor.setPower(topRightPower);
        bottomLeftDriveMotor.setPower(bottomLeftPower);
        bottomRightDriveMotor.setPower(bottomRightPower);
    }
}