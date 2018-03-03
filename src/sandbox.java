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
        byte b = 6;
        int a = 6;
        System.out.println(a == b);
    }

    public static byte[] toBytes(double d)
    {
        int rounded = (int)(d * 100000);
        System.out.println(Integer.toHexString(rounded));
        byte[] arr = {(byte)(rounded>>24),(byte)(rounded%(0x1000000)>>16),(byte)(rounded%(0x10000)>>8),(byte)(rounded%(0x100))};
        return arr;
    }

    public static double toDouble(byte[] arr)
    {
        int sum = 0;
        for(int i = 0; i < arr.length; i++)
        {
            int val = (arr[arr.length-1-i]) & 0xFF; //unsign
            sum += val << (8 * i);
        }
        return sum/100000.;
    }

}


