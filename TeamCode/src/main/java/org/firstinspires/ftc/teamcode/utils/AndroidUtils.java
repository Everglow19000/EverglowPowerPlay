package org.firstinspires.ftc.teamcode.utils;

import java.text.SimpleDateFormat;
import java.util.Date;

public class AndroidUtils {
    public static String timestampString(){
        return new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss_SSS").format(new Date());
    }
}
