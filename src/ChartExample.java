import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.DefaultXYDataset;

import javax.swing.*;

public class ChartExample
{

    public static void main(String[] args)
    {
        double[] x = { 1, 2, 3, 4, 5, 6 };
        double[] y = { 45, 89, 6, 32, 63, 12 };
        double[][] data = {x, y};
        DefaultXYDataset ds = new DefaultXYDataset();
        ds.addSeries("series1", data);

        JFreeChart chart = ChartFactory.createXYLineChart("test chart", "x", "y",
                ds, PlotOrientation.VERTICAL, true, true, false);
        ChartPanel cp = new ChartPanel(chart);
        JFrame frame = new JFrame("asdf");
        frame.setContentPane(cp);
        frame.setVisible(true);
        frame.pack();
    }

}


