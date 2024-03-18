package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {
    DcMotorEx motor;

    public Lift(DcMotorEx m) {
        this.motor = m;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public int getEncoderValue() {
        return this.motor.getCurrentPosition();
    }

    /**
     * only method that moves the lift
     * @param input positive values extends the string
     */
    public void move(double input){
        this.motor.setPower(input);
    }
}