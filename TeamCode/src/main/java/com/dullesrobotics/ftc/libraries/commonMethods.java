package com.dullesrobotics.ftc.libraries;

/**
 * Created by Kenneth on 1/8/2017.
 */

public class commonMethods {
    public static void delay(long millis){
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() < start+millis){}
    }
}
