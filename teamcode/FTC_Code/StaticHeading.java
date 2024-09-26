package org.firstinspires.ftc.teamcode.FTC_Code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Static Heading")
public class StaticHeading extends LinearOpMode {
    private double integralSum = 0;
    private double Kp = PIDConstants.Kp;
    private double Ki = PIDConstants.Ki;
    private double Kd = PIDConstants.Kd;

    private Driver_settings drivetrain = new Driver_settings();

    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private IMU imu;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD )
        );
        imu.initialize(parameters);

        double referenceAngle = Math.toRadians(90); // Convert degrees to radians
        waitForStart();

        while (opModeIsActive()) {
            // Get the current angle from the IMU
            double currentAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

            // Update telemetry
            telemetry.addData("Target IMU Angle", referenceAngle);
            telemetry.addData("Current IMU Angle", currentAngle);

            // Calculate the control power using PID
            double power = PIDControl(referenceAngle, currentAngle);
            drivetrain.power(power);

            // Update telemetry display
            telemetry.update();
        }
    }

    private double PIDControl(double reference, double state) {
        double error = angleWrap(reference - state);
        telemetry.addData("Error", error);

        double deltaTime = timer.seconds();
        integralSum += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;

        lastError = error;
        timer.reset();

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}