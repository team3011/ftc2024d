package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystems.Telescope;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

import java.text.DecimalFormat;

@TeleOp(name = "TELE_RED")
public class Tele_Red extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean fromAuto = true;
        boolean isRed = true;

        //this section allows us to access telemetry data from a browser
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        GamepadEx myGamePad = new GamepadEx(gamepad1);
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
        Claw claw = new Claw(
                hardwareMap.get(Servo.class,"leftClaw"),
                hardwareMap.get(Servo.class,"rightClaw"),
                hardwareMap.get(TouchSensor.class,"clawSensor"),
                false);
        RevBlinkinLedDriver blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "servo");
        Arm arm = new Arm(shoulder, telescope, claw, lift, wrist, navx, blinkin, fromAuto, isRed);

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
        double left_t = gamepad1.left_trigger;
        double right_t = gamepad1.right_trigger;
        boolean endGameStarted = false;
        waitForStart();
        arm.goTo_Initialization();
        while(opModeIsActive()) {
            myGamePad.readButtons();
            left_y = zeroAnalogInput(gamepad1.left_stick_y);
            right_y = zeroAnalogInput(gamepad1.right_stick_y);
            left_x = zeroAnalogInput(gamepad1.left_stick_x);
            right_x = zeroAnalogInput(gamepad1.right_stick_x);
            left_t = -zeroAnalogInput(gamepad1.left_trigger);
            right_t = zeroAnalogInput(gamepad1.right_trigger);



            if (!myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.wasJustReleased(GamepadKeys.Button.A)) {
                arm.goTo_Pickup(false);
            }
            if (!myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.wasJustReleased(GamepadKeys.Button.X)) {
                arm.goTo_Stow(false);
            }
            if (!myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.wasJustReleased(GamepadKeys.Button.Y)) {
                arm.goTo_dropOff(false);
                TelemetryData.liftStage = -3;
            }

            wrist.update();

            if (left_t != 0) {
                arm.prepForLift(left_t);
                endGameStarted = true;
                //shoulder.update(0.3);
                //telescope.update(0.3);
            } else {
                if (right_y != 0) {
                    shoulder.move(right_y, true);
                } else {
                    shoulder.update(1);
                }

                if (left_y != 0) {
                    telescope.move(left_y, true);
                } else {
                    telescope.update(1);
                }
            }


            double modifier = 0;
            if (endGameStarted) {
                if (TelemetryData.shoulder_current > 4000) {
                    if (TelemetryData.shoulder_current > 5000) {
                        modifier = 1;
                    } else {
                        modifier = 1 - (5000 - TelemetryData.shoulder_current) / 1000. + .1;
                    }
                } else if (TelemetryData.telescope_current > 1000) {
                    if (TelemetryData.telescope_current > 1500) {
                        modifier = 1;
                    } else {
                        modifier = 1 - (1500 - TelemetryData.telescope_current) / 500. + .1;
                    }
                }
                lift.move(modifier);
            }

            telemetry.addData("lift stage", TelemetryData.liftStage);
            telemetry.addData("lift power",modifier);
            telemetry.addData("lift current", TelemetryData.lift_current);
            telemetry.addData("shoulder current", TelemetryData.shoulder_current);
            telemetry.addData("shoulder power", TelemetryData.shoulder_power);
            telemetry.addData("shoulder target", TelemetryData.shoulder_target);
            telemetry.addData("telescope current", TelemetryData.telescope_current);
            telemetry.addData("shoulder position", TelemetryData.shoulder_position);
            telemetry.addData("telescope position", TelemetryData.telescope_position);
            telemetry.addData("telescope target", TelemetryData.telescope_target);
            telemetry.addData("lift position", lift.getEncoderValue());
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
