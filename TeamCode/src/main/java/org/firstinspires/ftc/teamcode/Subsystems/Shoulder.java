package org.firstinspires.ftc.teamcode.Subsystems;


/*
Instructions on feedforward PID control can be found here:
https://www.youtube.com/watch?v=E6H6Nqe6qJo
 */


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Telescope;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class Shoulder {
    private DcMotorEx motor;
    private TouchSensor touch;
    private PIDController controller;

    /**
     * Class constructor
     *
     * @param m        motor obj
     * @param t        touch sensor obj
     * @param fromAuto was the constructor called from auto or tele
     */
    public Shoulder(DcMotorEx m, TouchSensor t, boolean fromAuto) {
        this.motor = m;
        this.touch = t;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setDirection(DcMotorEx.Direction.REVERSE);
        this.controller = new PIDController(RC_Shoulder.kP, RC_Shoulder.kI, RC_Shoulder.kD);
    }

    /**
     * only method that moves the shoulder
     * @param input positive numbers move the shoulder up
     * @param fromJoystick did this command to move come from a joystick
     */
    public void move(double input, boolean fromJoystick) {
        TelemetryData.shoulder_position = this.motor.getCurrentPosition();
        TelemetryData.shoulder_current = this.motor.getCurrent(CurrentUnit.MILLIAMPS);
        if (fromJoystick  && input != 0){
            TelemetryData.shoulder_target = TelemetryData.shoulder_position;
        }

        if (input > 0) {
            if (TelemetryData.shoulder_position > RC_Shoulder.maxOut) {
                input = 0;
            } else if (TelemetryData.shoulder_position > 1700) {
                input = .1 + calcFeedForward();
            }
        } else if (input < 0) {
            if (this.touch.isPressed()) {
                input = 0;
                this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (TelemetryData.shoulder_position < 500) {
                //input = -0.01; // -.05 + calcFeedForward();

            }
        } else {
            input = calcFeedForward();
            if (this.touch.isPressed()) {
                input = 0;
                this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        TelemetryData.shoulder_power = input;
        this.motor.setPower(input);
    }

    public void update(double lim){
        move(calcPower(lim),false);
    }

    public void setPosition(int t){
        TelemetryData.shoulder_target = t;
    }

    private double calcPower(double lim) {
        double power = 0;
        this.controller.setPID(RC_Shoulder.kP, RC_Shoulder.kI, RC_Shoulder.kD);
        TelemetryData.shoulder_position = this.motor.getCurrentPosition();
        double pid = this.controller.calculate(TelemetryData.shoulder_position, TelemetryData.shoulder_target);
        pid = pid/10;
        TelemetryData.shoulder_pid = limiter(pid,lim);
        double ff = calcFeedForward();
        power = limiter(pid + ff,lim);
        return power;
    }

    /**
     * this will limit the input to a range of -limiter to limiter
     * @param input the value to be limited
     * @param limiter the max value the input can be
     * @return the limited input
     */
    private double limiter(double input, double limiter){
        if (input > limiter) {
            input = limiter;
        } else if (input < -limiter) {
            input = -limiter;
        }
        return input;
    }

    /**
     * calculate the feedforward value by taking the cosine of the target angle relative to the ground
     * This works because cos() represents the ratio between the adjacent side of a right triangle and
     * the hypotenuse of the triangle. Whenever the arm is extended straight out (0 degrees),
     * the value of the cosine function is at its maximum (1). This is the point where the system
     * demands the most torque to hold its weight. Whenever we want the arm to be oriented straight
     * up or down (90 degrees), the arm does not require any torque to hold its weight. This matches
     * our feedforward controller, as cos(90 degrees) is 0.
     * By using a nonlinear feedforward controller, we can improve the reliability of our control
     * system and improve the performance of our system.
     *
     * @return the calculated feedforward value
     */
    private double calcFeedForward(){
        double temp = TelemetryData.shoulder_position / RC_Shoulder.ticksFor90 + RC_Shoulder.startAngle;
        return Math.cos(Math.toRadians(temp)) * RC_Shoulder.kF;
    }
}
