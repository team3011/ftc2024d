package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_Wrist;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystems.Telescope;

import org.firstinspires.ftc.teamcode.RobotConstants.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

import java.text.DecimalFormat;


@Autonomous(name = "SubSystem_Test")
public class SubSystem_Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        boolean fromAuto = true;
        boolean isRed = true;

        //this section allows us to access telemetry data from a browser
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        AHRS navx = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);

        Telescope telescope = new Telescope(
                hardwareMap.get(DcMotorEx.class, "telescope"),
                hardwareMap.get(TouchSensor.class, "telescopeSensor"),
                true);
        Wrist wrist = new Wrist(
                hardwareMap.get(Servo.class, "left"),
                hardwareMap.get(Servo.class, "right"),
                true);
        Shoulder shoulder = new Shoulder(
                hardwareMap.get(DcMotorEx.class, "shoulder"),
                hardwareMap.get(TouchSensor.class, "shoulderSensor"),
                true);
        Lift lift = new Lift(
                hardwareMap.get(DcMotorEx.class, "lift"));

        boolean connected = navx.isConnected();
        telemetry.addData("1 navX-Device", connected ?
                "Connected" : "Disconnected" );
        String gyrocal, magcal, yaw, pitch, roll, compass_heading;
        String fused_heading, ypr, cf, motion;
        DecimalFormat df = new DecimalFormat("#.##");

        double left_y = gamepad1.left_stick_y;
        double right_y = gamepad1.right_stick_y;
        double left_x = gamepad1.left_stick_x;
        double right_x = gamepad1.right_stick_x;
        waitForStart();
        wrist.setTarget(RC_Wrist.stowPos,RC_Wrist.stowTime);
        while(opModeIsActive()) {
            left_y = zeroAnalogInput(gamepad1.left_stick_y);
            right_y = zeroAnalogInput(gamepad1.right_stick_y);
            left_x = zeroAnalogInput(gamepad1.left_stick_x);
            right_x = zeroAnalogInput(gamepad1.right_stick_x);

            wrist.update();

            shoulder.move(right_y, true);
            telescope.move(left_y, true);
            lift.move(right_x);

            if (TelemetryData.shoulder_current > 2250) {
                double modifier = 1 - (2550 - TelemetryData.shoulder_current) / 300. + .1;
                lift.move(modifier);
            } else if (TelemetryData.telescope_current > 700) {
                double modifier = 1 - (900 - TelemetryData.telescope_current) / 200.;
                lift.move(modifier);
            } else {
                lift.move(0);
            }




            telemetry.addData("shoulder position", TelemetryData.shoulder_position);
            telemetry.addData("shoulder power", TelemetryData.shoulder_power);
            telemetry.addData("telescope position", TelemetryData.telescope_position);
            telemetry.addData("telescope target", TelemetryData.telescope_target);
            telemetry.update();
        }

    }

    /**
     * removes the analog drift
     * @param input
     * @return
     */
    private double zeroAnalogInput(double input){
        if (Math.abs(input) < RobotConstants.analogTol){
            input = 0;
        } else if (input > 0) {
            input -= RobotConstants.analogTol;
        } else {
            input += RobotConstants.analogTol;
        }
        return input;
    }
}