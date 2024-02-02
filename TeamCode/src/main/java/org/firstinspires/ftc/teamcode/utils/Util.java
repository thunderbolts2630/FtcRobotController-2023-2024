package org.firstinspires.ftc.teamcode.utils;

public class Util {
    //returns the max/ min is the value is too big/small
    public static double calmp(double value,double max,double min){
        return Math.min(Math.max(value,min),max);
    }
    public static double cosInDegrees(double angle) {
        return Math.cos(angle * Math.PI / 180);
    }

    public static double sinInDegrees(double angle) {
        return Math.sin(angle * Math.PI / 180);
    }

    public static double aCosInDegrees(double ratio) {
        return Math.acos(ratio) / Math.PI * 180;
    }
}