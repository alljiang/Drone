import com.fazecast.jSerialComm.SerialPort;
import com.sun.org.apache.xpath.internal.SourceTree;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.DefaultXYDataset;

import javax.swing.*;
import java.util.ArrayList;
import java.util.Arrays;

public class sandbox
{

    public static void main(String[] args)
    {
        double kp = 100;
        double ki = .03;
        double kd = 1200;
        byte[] holder = new byte[]{
                (byte) (kp < 0 ? 1 : 0), (byte) ((int) (Math.abs(kp) * 100000) >> 24), (byte) ((int) (Math.abs(kp) * 100000) >> 16 % (0x1000000)),
                (byte) ((int) (Math.abs(kp) * 100000) % (0x10000) >> 8), (byte) ((int) (Math.abs(kp) * 100000) % (0x100)),
                (byte) (ki < 0 ? 1 : 0), (byte) ((int) (Math.abs(ki) * 100000) >> 24), (byte) ((int) (Math.abs(ki) * 100000) >> 16 % (0x1000000)),
                (byte) ((int) (Math.abs(ki) * 100000) % (0x10000) >> 8), (byte) ((int) (Math.abs(ki) * 100000) % (0x100)),
                (byte) (kd < 0 ? 1 : 0), (byte) ((int) (Math.abs(kd) * 100000) >> 24), (byte) ((int) (Math.abs(kd) * 100000) >> 16 % (0x1000000)),
                (byte) ((int) (Math.abs(kd) * 100000) % (0x10000) >> 8), (byte) ((int) (Math.abs(kd) * 100000) % (0x100))
        };
        long kp_sign = unsign(holder[0]) == 0 ? 1 : -1;
        long kp_1 = ((long)unsign(holder[1])<<24);
        long kp_2 = ((long)unsign(holder[2])<<16);
        long kp_3 = ((long)unsign(holder[3])<<8);
        long kp_4 = ((long)unsign(holder[4])<<0);
        kp = kp_sign*(kp_1 + kp_2 + kp_3 + kp_4) / 100000.;

        long ki_sign = holder[5] == 0 ? 1 : -1;
        long ki_1 = ((long)holder[6]<<24);
        long ki_2 = ((long)holder[7]<<16);
        long ki_3 = ((long)holder[8]<<8);
        long ki_4 = ((long)holder[9]<<0);
        ki = ki_sign*(ki_1 + ki_2 + ki_3 + ki_4) / 100000.;

        long kd_sign = holder[10] == 0 ? 1 : -1;
        long kd_1 = ((long)holder[11]<<24);
        long kd_2 = ((long)holder[12]<<16);
        long kd_3 = ((long)holder[13]<<8);
        long kd_4 = ((long)holder[14]<<0);
        kd = kd_sign*(kd_1 + kd_2 + kd_3 + kd_4) / 100000.;
        System.out.println(kp);
        System.out.println(ki);
        System.out.println(kd);
    }

    public static int unsign(byte b)
    {
        return b & 0xFF;
    }

}


