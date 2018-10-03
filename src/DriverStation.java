import com.fazecast.jSerialComm.SerialPort;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.DefaultXYDataset;
import org.lwjgl.LWJGLException;
import org.lwjgl.input.Controller;
import org.lwjgl.input.Controllers;

import javax.swing.*;
import javax.swing.text.DefaultCaret;
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.*;

public class DriverStation
{

    private JPanel panelMain;
    private JButton Enable_Status; //Enabled Color = 00ff02 | Disabled Color = FF0007
    private JLabel ElapsedTimeLabel;
    private JTabbedPane TabPane;
    private JTextField YawP;
    private JTextField YawI;
    private JTextField YawD;
    private JButton applyChangesButton;
    private JTextField PitchP;
    private JTextField PitchI;
    private JTextField PitchD;
    private JTextField RollP;
    private JTextField RollI;
    private JTextField RollD;
    private JLabel YPR_Picture;
    private JComboBox COMCombo;
    private JProgressBar XAxis;
    private JProgressBar YAxis;
    private JProgressBar RXAxis;
    private JProgressBar RYAxis;
    private double XAxisVal = 0;
    private double YAxisVal = 0;
    private double RXAxisVal = 0;
    private double RYAxisVal = 0;
    private double POVXVal = 0;
    private double POVYVal = 0;
    private double POVXValLast = 0;
    private double POVYValLast = 0;
    private JTextPane Console;
    private JComboBox ControllerCombo;
    private JPanel BtnStatus0; //Off = 4B4B4B
    private JPanel BtnStatus1; //On = 64ff00
    private JPanel BtnStatus2;
    private JPanel BtnStatus3;
    private JPanel BtnStatus4;
    private JPanel BtnStatus5;
    private JPanel BtnStatus6;
    private JPanel BtnStatus7;
    private JPanel BtnStatus8;
    private JPanel BtnStatus9;
    private JPanel BtnStatus10;
    private JPanel BtnStatus11;
    private JPanel BtnStatus12;
    private boolean Btn0; //Y
    private boolean Btn1; //B
    private boolean Btn2; //A
    private boolean Btn3; //X
    private boolean Btn4; //LB
    private boolean Btn5; //RB
    private boolean Btn6; //LTRIGGER
    private boolean Btn7; //RTRIGGER
    private boolean Btn8; //BACKBTN
    private boolean Btn9; //STARTBTN
    private boolean Btn10; //LCLICK
    private boolean Btn11; //RCLICK
    private boolean Btn12; //HOMEBTN
    private boolean Btn4Last = false;
    private boolean Btn5Last = false;
    private boolean Btn6Last = false;
    private boolean Btn7Last = false;
    private JProgressBar POVXAxis;
    private JProgressBar POVYAxis;
    private JLabel CurrentYawI;
    private JLabel CurrentYawD;
    private JLabel CurrentPitchP;
    private JLabel CurrentPitchI;
    private JLabel CurrentPitchD;
    private JLabel CurrentRollP;
    private JLabel CurrentRollI;
    private JLabel CurrentRollD;
    private JLabel CurrentYawP;
    private JScrollPane ConsoleScrollPane;
    private JPanel DriverPanel;
    private JPanel PIDPanel;
    private JPanel ControllerPanel;
    private JSlider MotorController0;
    private JSlider MotorController1;
    private JSlider MotorController2;
    private JSlider MotorController3;
    private JLabel Speed0;
    private JLabel Speed1;
    private JLabel Speed2;
    private JLabel Speed3;
    private JLabel M1;
    private JLabel M0;
    private JLabel M3;
    private JLabel M2;
    private JLabel currentYaw;
    private JLabel currentPitch;
    private JLabel currentRoll;
    private JPanel controllerWarningPanel;
    private JPanel Motors;
    private JButton mpuZeroButton;
    private JPanel GraphPanel;
    private JLabel BatteryVoltage;
    private JButton resetButton;
    SerialPort port = null;
    boolean droneEnabled = false;
    boolean continueClock = false;
    long startElapsedTime = 0;
    double yawkp, yawki, yawkd;
    double kp, ki, kd;
    double rollkp, rollki, rollkd;
    int baudRate = 57600;
    int selectedControllerPort = 0;
    ArrayDeque<Double> rollStorage = new ArrayDeque<>();
    ArrayDeque<Double> pitchStorage = new ArrayDeque<>();
    JFrame graphFrame = new JFrame("PID Graph");

    PrintWriter pw = new PrintWriter(new File("data.out"));

    public DriverStation() throws FileNotFoundException
    {
        $$$setupUI$$$();
        Enable_Status.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
                if (!droneEnabled) enable();
                else disable();
            }
        });
        ControllerCombo.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
                printToConsole("Set controller to " + ControllerCombo.getItemAt(ControllerCombo.getSelectedIndex()));
            }
        });
        applyChangesButton.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
                String yp = YawP.getText();
                String yi = YawI.getText();
                String yd = YawD.getText();
                String pp = PitchP.getText();
                String pi = PitchI.getText();
                String pd = PitchD.getText();
                String rp = PitchP.getText();
                String ri = PitchI.getText();
                String rd = PitchD.getText();
                if (yp.length() > 0) yawkp = Double.parseDouble(yp);
                if (yi.length() > 0) yawki = Double.parseDouble(yi);
                if (yd.length() > 0) yawkd = Double.parseDouble(yd);
                if (pp.length() > 0) kp = Double.parseDouble(pp);
                if (pi.length() > 0) ki = Double.parseDouble(pi);
                if (pd.length() > 0) kd = Double.parseDouble(pd);
                if (rp.length() > 0) rollkp = Double.parseDouble(rp);
                if (ri.length() > 0) rollki = Double.parseDouble(ri);
                if (rd.length() > 0) rollkd = Double.parseDouble(rd);
                YawP.setText("");
                YawI.setText("");
                YawD.setText("");
                PitchP.setText("");
                PitchI.setText("");
                PitchD.setText("");
                RollP.setText("");
                RollI.setText("");
                RollD.setText("");

                String text = updatePID();

                CurrentYawP.setText(yawkp + "");
                CurrentYawI.setText(yawki + "");
                CurrentYawD.setText(yawkd + "");
                CurrentPitchP.setText(kp + "");
                CurrentPitchI.setText(ki + "");
                CurrentPitchD.setText(kd + "");
                CurrentRollP.setText(rollkp + "");
                CurrentRollI.setText(rollki + "");
                CurrentRollD.setText(rollkd + "");
                try
                {
                    PrintWriter pw = new PrintWriter("PIDStorage.dat");
                    pw.println(text);
                    pw.close();
                } catch (FileNotFoundException e1)
                {
                    e1.printStackTrace();
                }
            }
        });

        try
        {
            BufferedReader br = new BufferedReader(new FileReader("PIDStorage.dat"));
            StringTokenizer st = new StringTokenizer(br.readLine());
            kp = Double.parseDouble(st.nextToken());
            ki = Double.parseDouble(st.nextToken());
            kd = Double.parseDouble(st.nextToken());
            yawkp = Double.parseDouble(st.nextToken());
            yawki = Double.parseDouble(st.nextToken());
            yawkd = Double.parseDouble(st.nextToken());
            rollkp = kp;
            rollki = ki;
            rollkd = kd;
            CurrentYawP.setText(yawkp + "");
            CurrentYawI.setText(yawki + "");
            CurrentYawD.setText(yawkd + "");
            CurrentPitchP.setText(kp + "");
            CurrentPitchI.setText(ki + "");
            CurrentPitchD.setText(kd + "");
            CurrentRollP.setText(rollkp + "");
            CurrentRollI.setText(rollki + "");
            CurrentRollD.setText(rollkd + "");
        } catch (Exception e)
        {
            e.printStackTrace();
        }
        resetButton.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
                port.closePort();
                port.openPort();
                port.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
                port.setBaudRate(baudRate);
            }
        });

        //set enter as a disable key
        Action disable = new AbstractAction()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
                if (droneEnabled)
                {
                    disable();
                }
            }
        };
        panelMain.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        panelMain.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        panelMain.getActionMap().put("disable", disable);
        COMCombo.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        COMCombo.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        COMCombo.getActionMap().put("disable", disable);
        ControllerCombo.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        ControllerCombo.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        ControllerCombo.getActionMap().put("disable", disable);
        PIDPanel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        PIDPanel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        PIDPanel.getActionMap().put("disable", disable);
        DriverPanel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        DriverPanel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        DriverPanel.getActionMap().put("disable", disable);
        controllerWarningPanel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        controllerWarningPanel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        controllerWarningPanel.getActionMap().put("disable", disable);
        TabPane.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        TabPane.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        TabPane.getActionMap().put("disable", disable);
        applyChangesButton.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        applyChangesButton.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        applyChangesButton.getActionMap().put("disable", disable);
        YawP.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        YawP.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        YawP.getActionMap().put("disable", disable);
        YawI.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        YawI.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        YawI.getActionMap().put("disable", disable);
        YawD.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        YawD.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        YawD.getActionMap().put("disable", disable);
        PitchP.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        PitchP.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        PitchP.getActionMap().put("disable", disable);
        PitchI.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        PitchI.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        PitchI.getActionMap().put("disable", disable);
        PitchD.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        PitchD.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        PitchD.getActionMap().put("disable", disable);
        RollP.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        RollP.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        RollP.getActionMap().put("disable", disable);
        RollI.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        RollI.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        RollI.getActionMap().put("disable", disable);
        RollD.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        RollD.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        RollD.getActionMap().put("disable", disable);
        mpuZeroButton.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "disable");
        mpuZeroButton.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "disable");
        mpuZeroButton.getActionMap().put("disable", disable);

        Console.setFocusable(false);
        DefaultCaret caret = (DefaultCaret) Console.getCaret();
        caret.setUpdatePolicy(DefaultCaret.ALWAYS_UPDATE);
        YPR_Picture.addMouseListener(new MouseAdapter()
        {
            @Override
            public void mouseClicked(MouseEvent e)
            {
                disable();
            }
        });

        RollP.setVisible(false);
        RollI.setVisible(false);
        RollD.setVisible(false);

        COMCombo.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
                try
                {
                    try
                    {
                        port.closePort();
                    } catch (Exception ee)
                    {
                    }
                    port = SerialPort.getCommPorts()[COMCombo.getSelectedIndex()];
                    System.out.println(port.getSystemPortName());
                    if (port.openPort())
                    {
                        port.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
                        port.setBaudRate(baudRate);
                        printToConsole("Set COM port to " + COMCombo.getItemAt(COMCombo.getSelectedIndex()));
                    }
                } catch (Exception ee)
                {
                    System.out.println("COMCombo ActionListener Error");
                }
            }
        });
        mpuZeroButton.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
                zeroMPU();
            }
        });
    }

    public static void main(String[] args) throws IOException
    {
        //create frame
        JFrame frame = new JFrame("Drone Driver Station");
        frame.setContentPane(new DriverStation().panelMain);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.pack();
        frame.getContentPane().setPreferredSize(new Dimension(1920, 1080));
        frame.setVisible(true);
    }

    int lastAmountofPorts;
    int lastAmountofControllers;

    private void createUIComponents()
    {
        XAxis = new JProgressBar();
        YAxis = new JProgressBar();
        RXAxis = new JProgressBar();
        RYAxis = new JProgressBar();
        POVXAxis = new JProgressBar();
        POVXAxis = new JProgressBar();
        POVYAxis = new JProgressBar();
        YawP = new JTextField();
        YawI = new JTextField();
        YawD = new JTextField();
        PitchP = new JTextField();
        PitchI = new JTextField();
        PitchD = new JTextField();
        RollP = new JTextField();
        RollI = new JTextField();
        RollD = new JTextField();
        CurrentYawP = new JLabel();
        CurrentYawI = new JLabel();
        CurrentYawD = new JLabel();
        CurrentPitchP = new JLabel();
        CurrentPitchI = new JLabel();
        CurrentPitchD = new JLabel();
        CurrentRollP = new JLabel();
        CurrentRollI = new JLabel();
        CurrentRollD = new JLabel();
        BtnStatus0 = new JPanel();
        BtnStatus1 = new JPanel();
        BtnStatus2 = new JPanel();
        BtnStatus3 = new JPanel();
        BtnStatus4 = new JPanel();
        BtnStatus5 = new JPanel();
        BtnStatus6 = new JPanel();
        BtnStatus7 = new JPanel();
        BtnStatus8 = new JPanel();
        BtnStatus9 = new JPanel();
        BtnStatus10 = new JPanel();
        BtnStatus11 = new JPanel();
        BtnStatus12 = new JPanel();
        GraphPanel = new JPanel();
        Speed0 = new JLabel("0");
        Speed1 = new JLabel("0");
        Speed2 = new JLabel("0");
        Speed3 = new JLabel("0");
        MotorController0 = new JSlider(0, 100, 0);
        MotorController1 = new JSlider(0, 100, 0);
        MotorController2 = new JSlider(0, 100, 0);
        MotorController3 = new JSlider(0, 100, 0);
        controllerWarningPanel = new JPanel();

        //Initialize Graph
        double[] x = {};
        double[] y = {};
        double[][] data = {x, y};
        DefaultXYDataset ds = new DefaultXYDataset();
        ds.addSeries("series1", data);

        JFreeChart chart = ChartFactory.createXYLineChart("PID Chart", "Axis", "Angle",
                ds, PlotOrientation.VERTICAL, true, true, false);
        ChartPanel cp = new ChartPanel(chart);
        graphFrame.setContentPane(cp);
        graphFrame.setVisible(false);
        graphFrame.pack();

        //COM Port Selection
        SerialPort[] portNames = SerialPort.getCommPorts();
        lastAmountofPorts = portNames.length;
        COMCombo = new JComboBox();
        ControllerCombo = new JComboBox();
        ControllerCombo.setMaximumRowCount(100);
        for (SerialPort sp : portNames)
        {
            COMCombo.addItem(sp.getSystemPortName());
        }
        if (portNames.length > 0)
        {
            port = SerialPort.getCommPorts()[0];
            if (port.openPort())
            {
//                port.setComPortTimeouts(SerialPort.TIMEOUT_SCANNER, 0, 0);
                port.setBaudRate(baudRate);
            }
        }
        //Controller Selection
        try
        {
            Controllers.create();
        } catch (LWJGLException e)
        {
            e.printStackTrace();
        }
        Controllers.poll();
        lastAmountofControllers = Controllers.getControllerCount();
        for (int i = 0; i < lastAmountofControllers; i++)
        {
            Controller controller = Controllers.getController(i);
            ControllerCombo.addItem(controller.getName());
        }
        //Start update loop
        Thread updateLoop = new Thread()
        {
            public void run()
            {
                try
                {
                    while (true)
                    {
                        update();
                        Thread.sleep(100);
                    }
                } catch (Exception e)
                {
                    e.printStackTrace();
                }
            }
        };
        Thread receiveLoop = new Thread()
        {
            public void run()
            {
                try
                {
                    while (true)
                    {
                        receive();
                        Thread.sleep(20);
                    }
                } catch (Exception e)
                {
                    e.printStackTrace();
                }
            }
        };
        updateLoop.start();
        receiveLoop.start();

    }

    private void enable()
    {
        Enable_Status.setForeground(new Color(0x00ff02));
        Enable_Status.setText("Enabled");
        droneEnabled = true;
        continueClock = true;
        startElapsedTime = System.currentTimeMillis();
        printToConsole("Drone Enabled");
        MotorController0.setValue(0);
        MotorController1.setValue(0);
        MotorController2.setValue(0);
        MotorController3.setValue(0);
        currentThrottle = 0;
        lastUpdateTime = System.currentTimeMillis();
        send(new byte[]{0, 1});
        send(new byte[]{3, 0, 0, 0, 0});
        pw.close();
        try
        {
            pw = new PrintWriter("data.out");
        } catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
        System.out.println("enable");
    }

    private void disable()
    {
        Enable_Status.setForeground(new Color(0xff0007));
        Enable_Status.setText("Disabled");
        droneEnabled = false;
        continueClock = false;
        printToConsole("Drone Disabled");
        MotorController0.setValue(0);
        MotorController1.setValue(0);
        MotorController2.setValue(0);
        MotorController3.setValue(0);
        send(new byte[]{0, 0});
        pw.close();
        updateGraph();
        System.out.println("disable");
        currentThrottle = 0;
    }

    double currentThrottle = 0;
    long lastUpdateTime = 0;

    private void drive()
    {
        long timeLength = System.currentTimeMillis() - lastUpdateTime;
        lastUpdateTime = System.currentTimeMillis();
        double toChange = timeLength * YAxisVal / ((YAxisVal > 0) ? 3000. : 2500.);
        currentThrottle += toChange;
        currentThrottle = Math.min(100, Math.max(0, currentThrottle));
        System.out.println(currentThrottle);
        send(new byte[]{0x2, (byte) currentThrottle, (byte) Math.round(RYAxisVal), (byte) Math.round(RXAxisVal), Btn6 ? (byte) 1 : (byte) 0, Btn7 ? (byte) 1 : (byte) 0});
    }

    int errorReportLoops = 5;
    int errorReportLoopsCount = 0;

    private void send(byte[] toSend)
    {
        if (toSend.length == 0) return;
        try
        {
            byte[] packet = concatenate(toSend, new byte[]{calculateChecksum(toSend)});
            if (port != null) port.writeBytes(packet, packet.length);
            System.out.println("Sent: " + Arrays.toString(toSend));
        } catch (Exception e)
        {
            if (errorReportLoopsCount++ == errorReportLoops)
            {
                errorReportLoopsCount = 0;
                System.out.println("Printwriter COM Port Error");
                e.printStackTrace();
            }
        }
    }

    private byte calculateChecksum(byte[] arr)
    {
        byte sum = 0;
        for (byte b : arr)
            sum += b;
        return sum;
    }

    private void receive()
    {
        try
        {
            if (port == null || port.bytesAvailable() == 0) return;
            byte[] cmd = new byte[1];
            port.readBytes(cmd, 1);
            int command = unsign(cmd[0]);
            if (command < 7 || command > 0xA) return;
            if (command == 0xA)
            {
                byte[] batteryValue = new byte[1];
                port.readBytes(batteryValue, 1);
                byte[] checkSum = new byte[1];
                port.readBytes(checkSum, 1);
                byte calculatedChecksum = (byte) (calculateChecksum(batteryValue) + command);
                if (calculatedChecksum != checkSum[0])
                {
                    flush();
                    return;
                }
                BatteryVoltage.setText("Battery Voltage: " + (batteryValue[0] / 10.) + "V");
            } else if (command == 9) //CURRENT MOTOR VALUES
            {
                byte[] motorValues = new byte[4];
                port.readBytes(motorValues, 4);
                byte[] checkSum = new byte[1];
                port.readBytes(checkSum, 1);
                byte calculatedChecksum = (byte) (calculateChecksum(motorValues) + command);
                if (calculatedChecksum != checkSum[0])
                {
                    flush();
                    return;
                }
                M0.setText("M0: " + motorValues[0]);
                M1.setText("M1: " + motorValues[1]);
                M2.setText("M2: " + motorValues[2]);
                M3.setText("M3: " + motorValues[3]);
            } else if (command == 8)//CURRENT MPU READINGS
            {
                byte[] readings = new byte[3];
                port.readBytes(readings, 3);
                byte[] checkSum = new byte[1];
                port.readBytes(checkSum, 1);
                byte calculatedChecksum = (byte) (calculateChecksum(readings) + command);
                if (calculatedChecksum != checkSum[0])
                {
                    flush();
                    return;
                }
                currentYaw.setText(readings[0] + "");
                currentPitch.setText(readings[1] + "");
                currentRoll.setText(readings[2] + "");
            } else if (command == 7) //MESSAGE INTO CONSOLE
            {
                byte[] sizeSigned = new byte[1];
                port.readBytes(sizeSigned, 1);
                int size = unsign(sizeSigned[0]);
                byte[] strArr = new byte[size];
                port.readBytes(strArr, size);
                byte[] checkSum = new byte[1];
                port.readBytes(checkSum, 1);
                byte calculatedChecksum = (byte) (cmd[0] + calculateChecksum(strArr) + size);
                if (calculatedChecksum != checkSum[0])
                {
                    flush();
                    return;
                }
                String s = "";
                for (byte b : strArr) s += (char) unsign(b);
                printToConsole(s);
            }
        } catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    private byte[] readBytes(int numToRead)
    {
        byte[] toReturn = new byte[numToRead];
        while (port.bytesAvailable() > 0)
        {
            port.readBytes(toReturn, 4);
        }
        return toReturn;
    }

    private void flush()
    {
        int toFlush = port.bytesAvailable();
        port.readBytes(new byte[toFlush], toFlush);
    }

    private byte[] concatenate(byte[] one, byte[] two)
    {
        int totalsize = one.length + two.length;
        byte[] toReturn = new byte[totalsize];
        int index = 0;
        for (byte b : one)
            toReturn[index++] = b;
        for (byte b : two)
            toReturn[index++] = b;
        return toReturn;
    }

    private int unsign(byte b)
    {
        return b & 0xFF;
    }

    private void update()
    {
        //Update Serial Port list
        SerialPort[] portNames = SerialPort.getCommPorts();
        if (portNames.length != lastAmountofPorts)
        {
            lastAmountofPorts = portNames.length;
            COMCombo.removeAllItems();
            for (SerialPort sp : portNames)
            {
                COMCombo.addItem(sp.getSystemPortName());
            }
        }

        //Update Controller List
        Controllers.poll();
        if (Controllers.getControllerCount() != lastAmountofControllers)
        {
            lastAmountofControllers = Controllers.getControllerCount();
            ControllerCombo.removeAllItems();
            for (int i = 0; i < lastAmountofControllers; i++)
            {
                ControllerCombo.addItem(Controllers.getController(i).getName());
            }
        }

        selectedControllerPort = ControllerCombo.getSelectedIndex();

        //Update Controller Readings
        Controllers.poll();
        Controller controller = Controllers.getController(selectedControllerPort);
        XAxis.setValue((int) (controller.getXAxisValue() * 10000));
        XAxisVal = controller.getXAxisValue() * 100;
        if (controller.getAxisCount() > 0) YAxis.setValue((int) (controller.getAxisValue(2) * -10000));
        if (controller.getAxisCount() > 0) YAxisVal = controller.getAxisValue(2) * -100;
        RXAxis.setValue((int) (controller.getZAxisValue() * 10000));
        RXAxisVal = controller.getZAxisValue() * 100;
        if (controller.getAxisCount() > 0) RYAxis.setValue((int) (controller.getAxisValue(0) * -10000));
        if (controller.getAxisCount() > 0) RYAxisVal = controller.getAxisValue(0) * -100;
        if (Math.abs(XAxisVal) == 100 || Math.abs(YAxisVal) == 100 || Math.abs(RXAxisVal) == 100 || Math.abs(RYAxisVal) == 100)
            controllerWarningPanel.setBackground(new Color(0xff0007));
        else
            controllerWarningPanel.setBackground(new Color(0x3C3F41));
        if (controller.getButtonCount() > 0) updateControllerButtons(controller);
        POVXAxis.setValue((int) (controller.getPovX()));
        POVXVal = controller.getPovX();
        POVYAxis.setValue((int) (controller.getPovY() * -1));
        POVYVal = controller.getPovY() * -1;

//        Send trim controls
        if (POVYVal == 1 && POVXValLast == 0 && POVYValLast == 0)
            send(new byte[]{5, 0});
        else if (POVXVal == 1 && POVXValLast == 0 && POVYValLast == 0)
            send(new byte[]{5, 1});
        else if (POVYVal == -1 && POVXValLast == 0 && POVYValLast == 0)
            send(new byte[]{5, 2});
        else if (POVXVal == -1 && POVXValLast == 0 && POVYValLast == 0)
            send(new byte[]{5, 3});
        POVXValLast = POVXVal;
        POVYValLast = POVYVal;

        if (Btn6 && !Btn6Last)
            send(new byte[]{6, 0});
        else if (Btn7 && !Btn7Last)
            send(new byte[]{6, 1});
        Btn4Last = Btn4;
        Btn5Last = Btn5;

        //Update Elapsed Time if enabled
        if (continueClock)
        {
            long ElapsedTime = System.currentTimeMillis() - startElapsedTime;
            int minutes = (int) (ElapsedTime / 1000 / 60);
            int seconds = (int) (ElapsedTime / 1000 % 60);
            int milliseconds = (int) (ElapsedTime % 1000);
            String milli = "" + milliseconds;
            String sec = "" + seconds;
            String min = "" + minutes;
            if (minutes < 10) min = "0" + min;
            if (seconds < 10) sec = "0" + sec;
            if (milliseconds < 10) milli = "0" + milli;
            if (milliseconds < 100) milli = "0" + milli;
            ElapsedTimeLabel.setText("Elapsed Time: " + min + ":" + sec + ":" + milli);
        }

        if (droneEnabled) drive();
            //motor testing
        else
        {
            byte[] toSend = new byte[]{0x3, (byte) MotorController0.getValue(), (byte) MotorController1.getValue(),
                    (byte) MotorController2.getValue(), (byte) MotorController3.getValue()};
            send(toSend);
        }

        if (Btn8) disable();
        if (Btn3) zeroMPU();

        Speed0.setText(MotorController0.getValue() + "");
        Speed1.setText(MotorController1.getValue() + "");
        Speed2.setText(MotorController2.getValue() + "");
        Speed3.setText(MotorController3.getValue() + "");
    }

    private void updateControllerButtons(Controller controller)
    {
        if (controller.isButtonPressed(0))
        {
            Btn0 = true;
            BtnStatus0.setBackground(new Color(0x64ff00));
        } else
        {
            Btn0 = false;
            BtnStatus0.setBackground(new Color(0x4B4B4B));
        }
        if (controller.isButtonPressed(1))
        {
            Btn1 = true;
            BtnStatus1.setBackground(new Color(0x64ff00));
        } else
        {
            Btn1 = false;
            BtnStatus1.setBackground(new Color(0x4B4B4B));
        }
        if (controller.isButtonPressed(2))
        {
            Btn2 = true;
            BtnStatus2.setBackground(new Color(0x64ff00));
        } else
        {
            Btn2 = false;
            BtnStatus2.setBackground(new Color(0x4B4B4B));
        }
        if (controller.isButtonPressed(3))
        {
            Btn3 = true;
            BtnStatus3.setBackground(new Color(0x64ff00));
        } else
        {
            Btn3 = false;
            BtnStatus3.setBackground(new Color(0x4B4B4B));
        }
        if (controller.isButtonPressed(4))
        {
            Btn4 = true;
            BtnStatus4.setBackground(new Color(0x64ff00));
        } else
        {
            Btn4 = false;
            BtnStatus4.setBackground(new Color(0x4B4B4B));
        }
        if (controller.isButtonPressed(5))
        {
            Btn5 = true;
            BtnStatus5.setBackground(new Color(0x64ff00));
        } else
        {
            Btn5 = false;
            BtnStatus5.setBackground(new Color(0x4B4B4B));
        }
        if (controller.isButtonPressed(6))
        {
            Btn6 = true;
            BtnStatus6.setBackground(new Color(0x64ff00));
        } else
        {
            Btn6 = false;
            BtnStatus6.setBackground(new Color(0x4B4B4B));
        }
        if (controller.isButtonPressed(7))
        {
            Btn7 = true;
            BtnStatus7.setBackground(new Color(0x64ff00));
        } else
        {
            Btn7 = false;
            BtnStatus7.setBackground(new Color(0x4B4B4B));
        }
        if (controller.isButtonPressed(8))
        {
            Btn8 = true;
            BtnStatus8.setBackground(new Color(0x64ff00));
        } else
        {
            Btn8 = false;
            BtnStatus8.setBackground(new Color(0x4B4B4B));
        }
        if (controller.isButtonPressed(9))
        {
            Btn9 = true;
            BtnStatus9.setBackground(new Color(0x64ff00));
        } else
        {
            Btn9 = false;
            BtnStatus9.setBackground(new Color(0x4B4B4B));
        }
        if (controller.isButtonPressed(10))
            {
            Btn10 = true;
            BtnStatus10.setBackground(new Color(0x64ff00));
        } else
        {
            Btn10 = false;
            BtnStatus10.setBackground(new Color(0x4B4B4B));
        }
        if (controller.isButtonPressed(11))
        {
            Btn11 = true;
            BtnStatus11.setBackground(new Color(0x64ff00));
        } else
        {
            Btn11 = false;
            BtnStatus11.setBackground(new Color(0x4B4B4B));
        }
        if (controller.isButtonPressed(12))
        {
            Btn12 = true;
            BtnStatus12.setBackground(new Color(0x64ff00));
        } else
        {
            Btn12 = false;
            BtnStatus12.setBackground(new Color(0x4B4B4B));
        }
    }

    private String updatePID()
    {
        String text = kp + " " + ki + " " + kd + " " + yawkp + " " + yawki + " " + yawkd;
        printToConsole("Set PID to: " + text);
        byte[] toSend = new byte[]{4,
                (byte) (kp < 0 ? 1 : 0), (byte) ((int) (Math.abs(kp) * 100000) >> 24), (byte) ((int) (Math.abs(kp) * 100000) >> 16 % (0x1000000)),
                (byte) ((int) (Math.abs(kp) * 100000) % (0x10000) >> 8), (byte) ((int) (Math.abs(kp) * 100000) % (0x100)),
                (byte) (ki < 0 ? 1 : 0), (byte) ((int) (Math.abs(ki) * 100000) >> 24), (byte) ((int) (Math.abs(ki) * 100000) >> 16 % (0x1000000)),
                (byte) ((int) (Math.abs(ki) * 100000) % (0x10000) >> 8), (byte) ((int) (Math.abs(ki) * 100000) % (0x100)),
                (byte) (kd < 0 ? 1 : 0), (byte) ((int) (Math.abs(kd) * 100000) >> 24), (byte) ((int) (Math.abs(kd) * 100000) >> 16 % (0x1000000)),
                (byte) ((int) (Math.abs(kd) * 100000) % (0x10000) >> 8), (byte) ((int) (Math.abs(kd) * 100000) % (0x100)),
                (byte) (yawkp < 0 ? 1 : 0), (byte) ((int) (Math.abs(yawkp) * 100000) >> 24), (byte) ((int) (Math.abs(yawkp) * 100000) >> 16 % (0x1000000)),
                (byte) ((int) (Math.abs(yawkp) * 100000) % (0x10000) >> 8), (byte) ((int) (Math.abs(yawkp) * 100000) % (0x100)),
                (byte) (yawki < 0 ? 1 : 0), (byte) ((int) (Math.abs(yawki) * 100000) >> 24), (byte) ((int) (Math.abs(yawki) * 100000) >> 16 % (0x1000000)),
                (byte) ((int) (Math.abs(yawki) * 100000) % (0x10000) >> 8), (byte) ((int) (Math.abs(yawki) * 100000) % (0x100)),
                (byte) (yawkd < 0 ? 1 : 0), (byte) ((int) (Math.abs(yawkd) * 100000) >> 24), (byte) ((int) (Math.abs(yawkd) * 100000) >> 16 % (0x1000000)),
                (byte) ((int) (Math.abs(yawkd) * 100000) % (0x10000) >> 8), (byte) ((int) (Math.abs(yawkd) * 100000) % (0x100))
        };
        send(toSend);
        return text;
    }

    private void updateGraph()
    {
        double[] rollX = new double[rollStorage.size()];
        double[] pitchX = new double[pitchStorage.size()];
        double[] rollY = new double[rollStorage.size()];
        double[] pitchY = new double[pitchStorage.size()];
        for (int i = 0; i < rollX.length; i++)
        {
            rollX[i] = i;
            rollY[i] = rollStorage.pollFirst();
        }
        for (int i = 0; i < pitchX.length; i++)
        {
            pitchX[i] = i;
            pitchY[i] = pitchStorage.pollFirst();
        }
        printToConsole(Arrays.toString(rollY));
        double[][] rollData = {rollX, rollY};
        double[][] pitchData = {pitchX, pitchY};
        DefaultXYDataset ds = new DefaultXYDataset();
        ds.addSeries("Roll", rollData);
        ds.addSeries("Pitch", pitchData);
        JFreeChart chart = ChartFactory.createXYLineChart("PID Chart", "Axis", "Angle",
                ds, PlotOrientation.VERTICAL, true, true, false);
        ChartPanel cp = new ChartPanel(chart);
        graphFrame.getContentPane().removeAll();
        graphFrame.getContentPane().add(cp);
        graphFrame.pack();
        rollStorage.clear();
        pitchStorage.clear();
    }

    private void zeroMPU()
    {
        for (int i = 0; i < 10; i++)
            send(new byte[]{0x1, 0x2, 0x3});
    }

    String ConsoleHistory = "";

    private void printToConsole(String toPrint)
    {
        try
        {
            Date date = new Date();
            DateFormat df = new SimpleDateFormat("hh:mm:ss");
            String timeStamp = df.format(date);
            ConsoleHistory += ("[" + timeStamp + "]" + " " + toPrint + "\n");
            Console.setText(ConsoleHistory);
            JScrollBar vertical = ConsoleScrollPane.getVerticalScrollBar();
            vertical.setValue(vertical.getMaximum());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Method generated by IntelliJ IDEA GUI Designer
     * >>> IMPORTANT!! <<<
     * DO NOT edit this method OR call it in your code!
     *
     * @noinspection ALL
     */
    private void $$$setupUI$$$()
    {
        createUIComponents();
        panelMain = new JPanel();
        panelMain.setLayout(new com.intellij.uiDesigner.core.GridLayoutManager(1, 2, new Insets(0, 0, 0, 0), -1, -1));
        Font panelMainFont = this.$$$getFont$$$(null, -1, -1, panelMain.getFont());
        if (panelMainFont != null) panelMain.setFont(panelMainFont);
        TabPane = new JTabbedPane();
        Font TabPaneFont = this.$$$getFont$$$("Marlett", Font.BOLD, 48, TabPane.getFont());
        if (TabPaneFont != null) TabPane.setFont(TabPaneFont);
        TabPane.setRequestFocusEnabled(true);
        TabPane.setTabLayoutPolicy(0);
        TabPane.setTabPlacement(4);
        panelMain.add(TabPane, new com.intellij.uiDesigner.core.GridConstraints(0, 1, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, 1, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, new Dimension(200, 200), null, 0, false));
        ControllerPanel = new JPanel();
        ControllerPanel.setLayout(new com.intellij.uiDesigner.core.GridLayoutManager(21, 5, new Insets(0, 0, 0, 0), -1, -1));
        TabPane.addTab("Controller", ControllerPanel);
        final JLabel label1 = new JLabel();
        Font label1Font = this.$$$getFont$$$(null, Font.BOLD, 26, label1.getFont());
        if (label1Font != null) label1.setFont(label1Font);
        label1.setText("Axes");
        ControllerPanel.add(label1, new com.intellij.uiDesigner.core.GridConstraints(0, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final com.intellij.uiDesigner.core.Spacer spacer1 = new com.intellij.uiDesigner.core.Spacer();
        ControllerPanel.add(spacer1, new com.intellij.uiDesigner.core.GridConstraints(0, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, 1, null, null, null, 0, false));
        final com.intellij.uiDesigner.core.Spacer spacer2 = new com.intellij.uiDesigner.core.Spacer();
        ControllerPanel.add(spacer2, new com.intellij.uiDesigner.core.GridConstraints(20, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_VERTICAL, 1, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, null, null, null, 0, false));
        final JLabel label2 = new JLabel();
        Font label2Font = this.$$$getFont$$$(null, -1, 18, label2.getFont());
        if (label2Font != null) label2.setFont(label2Font);
        label2.setText("0: X Axis");
        ControllerPanel.add(label2, new com.intellij.uiDesigner.core.GridConstraints(1, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label3 = new JLabel();
        Font label3Font = this.$$$getFont$$$(null, -1, 18, label3.getFont());
        if (label3Font != null) label3.setFont(label3Font);
        label3.setText("1: Y Axis");
        ControllerPanel.add(label3, new com.intellij.uiDesigner.core.GridConstraints(2, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label4 = new JLabel();
        Font label4Font = this.$$$getFont$$$(null, -1, 18, label4.getFont());
        if (label4Font != null) label4.setFont(label4Font);
        label4.setText("2: RX Axis");
        ControllerPanel.add(label4, new com.intellij.uiDesigner.core.GridConstraints(3, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label5 = new JLabel();
        Font label5Font = this.$$$getFont$$$(null, -1, 18, label5.getFont());
        if (label5Font != null) label5.setFont(label5Font);
        label5.setText("3: RY Axis");
        ControllerPanel.add(label5, new com.intellij.uiDesigner.core.GridConstraints(4, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        XAxis = new JProgressBar();
        XAxis.setMaximum(10000);
        XAxis.setMinimum(-10000);
        ControllerPanel.add(XAxis, new com.intellij.uiDesigner.core.GridConstraints(1, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        YAxis.setMaximum(10000);
        YAxis.setMinimum(-10000);
        ControllerPanel.add(YAxis, new com.intellij.uiDesigner.core.GridConstraints(2, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        RXAxis.setMaximum(10000);
        RXAxis.setMinimum(-10000);
        ControllerPanel.add(RXAxis, new com.intellij.uiDesigner.core.GridConstraints(3, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        RYAxis.setMaximum(10000);
        RYAxis.setMinimum(-10000);
        ControllerPanel.add(RYAxis, new com.intellij.uiDesigner.core.GridConstraints(4, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label6 = new JLabel();
        Font label6Font = this.$$$getFont$$$(null, Font.BOLD, 26, label6.getFont());
        if (label6Font != null) label6.setFont(label6Font);
        label6.setText("Buttons");
        ControllerPanel.add(label6, new com.intellij.uiDesigner.core.GridConstraints(7, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label7 = new JLabel();
        label7.setText(" ");
        ControllerPanel.add(label7, new com.intellij.uiDesigner.core.GridConstraints(6, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label8 = new JLabel();
        label8.setText(" ");
        ControllerPanel.add(label8, new com.intellij.uiDesigner.core.GridConstraints(5, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label9 = new JLabel();
        Font label9Font = this.$$$getFont$$$(null, -1, 18, label9.getFont());
        if (label9Font != null) label9.setFont(label9Font);
        label9.setText("0: ");
        ControllerPanel.add(label9, new com.intellij.uiDesigner.core.GridConstraints(8, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label10 = new JLabel();
        Font label10Font = this.$$$getFont$$$(null, -1, 18, label10.getFont());
        if (label10Font != null) label10.setFont(label10Font);
        label10.setText("1: ");
        ControllerPanel.add(label10, new com.intellij.uiDesigner.core.GridConstraints(9, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label11 = new JLabel();
        Font label11Font = this.$$$getFont$$$(null, -1, 18, label11.getFont());
        if (label11Font != null) label11.setFont(label11Font);
        label11.setText("2: ");
        ControllerPanel.add(label11, new com.intellij.uiDesigner.core.GridConstraints(10, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label12 = new JLabel();
        Font label12Font = this.$$$getFont$$$(null, -1, 18, label12.getFont());
        if (label12Font != null) label12.setFont(label12Font);
        label12.setText("3: ");
        ControllerPanel.add(label12, new com.intellij.uiDesigner.core.GridConstraints(11, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label13 = new JLabel();
        Font label13Font = this.$$$getFont$$$(null, -1, 18, label13.getFont());
        if (label13Font != null) label13.setFont(label13Font);
        label13.setText("4: ");
        ControllerPanel.add(label13, new com.intellij.uiDesigner.core.GridConstraints(12, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label14 = new JLabel();
        Font label14Font = this.$$$getFont$$$(null, -1, 18, label14.getFont());
        if (label14Font != null) label14.setFont(label14Font);
        label14.setText("5:");
        ControllerPanel.add(label14, new com.intellij.uiDesigner.core.GridConstraints(13, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label15 = new JLabel();
        Font label15Font = this.$$$getFont$$$(null, -1, 18, label15.getFont());
        if (label15Font != null) label15.setFont(label15Font);
        label15.setText("6:");
        ControllerPanel.add(label15, new com.intellij.uiDesigner.core.GridConstraints(14, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        BtnStatus1.setBackground(new Color(-11842741));
        ControllerPanel.add(BtnStatus1, new com.intellij.uiDesigner.core.GridConstraints(9, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        BtnStatus2.setBackground(new Color(-11842741));
        ControllerPanel.add(BtnStatus2, new com.intellij.uiDesigner.core.GridConstraints(10, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        BtnStatus0.setBackground(new Color(-11842741));
        ControllerPanel.add(BtnStatus0, new com.intellij.uiDesigner.core.GridConstraints(8, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        BtnStatus3.setBackground(new Color(-11842741));
        ControllerPanel.add(BtnStatus3, new com.intellij.uiDesigner.core.GridConstraints(11, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        BtnStatus4.setBackground(new Color(-11842741));
        ControllerPanel.add(BtnStatus4, new com.intellij.uiDesigner.core.GridConstraints(12, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        BtnStatus5.setBackground(new Color(-11842741));
        ControllerPanel.add(BtnStatus5, new com.intellij.uiDesigner.core.GridConstraints(13, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        BtnStatus6.setBackground(new Color(-11842741));
        ControllerPanel.add(BtnStatus6, new com.intellij.uiDesigner.core.GridConstraints(14, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        final JLabel label16 = new JLabel();
        Font label16Font = this.$$$getFont$$$(null, -1, 18, label16.getFont());
        if (label16Font != null) label16.setFont(label16Font);
        label16.setText("8:");
        ControllerPanel.add(label16, new com.intellij.uiDesigner.core.GridConstraints(9, 3, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label17 = new JLabel();
        Font label17Font = this.$$$getFont$$$(null, -1, 18, label17.getFont());
        if (label17Font != null) label17.setFont(label17Font);
        label17.setText("7:");
        ControllerPanel.add(label17, new com.intellij.uiDesigner.core.GridConstraints(8, 3, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label18 = new JLabel();
        Font label18Font = this.$$$getFont$$$(null, -1, 18, label18.getFont());
        if (label18Font != null) label18.setFont(label18Font);
        label18.setText("9:");
        ControllerPanel.add(label18, new com.intellij.uiDesigner.core.GridConstraints(10, 3, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label19 = new JLabel();
        Font label19Font = this.$$$getFont$$$(null, -1, 18, label19.getFont());
        if (label19Font != null) label19.setFont(label19Font);
        label19.setText("10: ");
        ControllerPanel.add(label19, new com.intellij.uiDesigner.core.GridConstraints(11, 3, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label20 = new JLabel();
        Font label20Font = this.$$$getFont$$$(null, -1, 18, label20.getFont());
        if (label20Font != null) label20.setFont(label20Font);
        label20.setText("11: ");
        ControllerPanel.add(label20, new com.intellij.uiDesigner.core.GridConstraints(12, 3, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label21 = new JLabel();
        Font label21Font = this.$$$getFont$$$(null, -1, 18, label21.getFont());
        if (label21Font != null) label21.setFont(label21Font);
        label21.setText("12: ");
        ControllerPanel.add(label21, new com.intellij.uiDesigner.core.GridConstraints(13, 3, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label22 = new JLabel();
        label22.setText(" ");
        ControllerPanel.add(label22, new com.intellij.uiDesigner.core.GridConstraints(15, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label23 = new JLabel();
        label23.setText(" ");
        ControllerPanel.add(label23, new com.intellij.uiDesigner.core.GridConstraints(16, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label24 = new JLabel();
        Font label24Font = this.$$$getFont$$$(null, Font.BOLD, 26, label24.getFont());
        if (label24Font != null) label24.setFont(label24Font);
        label24.setText("POV");
        ControllerPanel.add(label24, new com.intellij.uiDesigner.core.GridConstraints(17, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label25 = new JLabel();
        Font label25Font = this.$$$getFont$$$(null, -1, 18, label25.getFont());
        if (label25Font != null) label25.setFont(label25Font);
        label25.setText("X Axis ");
        ControllerPanel.add(label25, new com.intellij.uiDesigner.core.GridConstraints(18, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label26 = new JLabel();
        Font label26Font = this.$$$getFont$$$(null, -1, 18, label26.getFont());
        if (label26Font != null) label26.setFont(label26Font);
        label26.setText("Y Axis");
        ControllerPanel.add(label26, new com.intellij.uiDesigner.core.GridConstraints(19, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        POVXAxis.setMaximum(1);
        POVXAxis.setMinimum(-1);
        ControllerPanel.add(POVXAxis, new com.intellij.uiDesigner.core.GridConstraints(18, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        POVYAxis.setMaximum(1);
        POVYAxis.setMinimum(-1);
        ControllerPanel.add(POVYAxis, new com.intellij.uiDesigner.core.GridConstraints(19, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final com.intellij.uiDesigner.core.Spacer spacer3 = new com.intellij.uiDesigner.core.Spacer();
        ControllerPanel.add(spacer3, new com.intellij.uiDesigner.core.GridConstraints(15, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, 1, null, null, null, 0, false));
        BtnStatus7.setBackground(new Color(-11842741));
        ControllerPanel.add(BtnStatus7, new com.intellij.uiDesigner.core.GridConstraints(8, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        BtnStatus8.setBackground(new Color(-11842741));
        ControllerPanel.add(BtnStatus8, new com.intellij.uiDesigner.core.GridConstraints(9, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        BtnStatus9.setBackground(new Color(-11842741));
        ControllerPanel.add(BtnStatus9, new com.intellij.uiDesigner.core.GridConstraints(10, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        BtnStatus10.setBackground(new Color(-11842741));
        ControllerPanel.add(BtnStatus10, new com.intellij.uiDesigner.core.GridConstraints(11, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        BtnStatus11.setBackground(new Color(-11842741));
        ControllerPanel.add(BtnStatus11, new com.intellij.uiDesigner.core.GridConstraints(12, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        BtnStatus12.setBackground(new Color(-11842741));
        ControllerPanel.add(BtnStatus12, new com.intellij.uiDesigner.core.GridConstraints(13, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        PIDPanel = new JPanel();
        PIDPanel.setLayout(new com.intellij.uiDesigner.core.GridLayoutManager(45, 9, new Insets(0, 0, 0, 0), -1, -1));
        Font PIDPanelFont = this.$$$getFont$$$(null, -1, -1, PIDPanel.getFont());
        if (PIDPanelFont != null) PIDPanel.setFont(PIDPanelFont);
        TabPane.addTab("PID Tuning", PIDPanel);
        final JLabel label27 = new JLabel();
        Font label27Font = this.$$$getFont$$$(null, Font.BOLD, 36, label27.getFont());
        if (label27Font != null) label27.setFont(label27Font);
        label27.setText("Yaw");
        PIDPanel.add(label27, new com.intellij.uiDesigner.core.GridConstraints(0, 0, 19, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label28 = new JLabel();
        Font label28Font = this.$$$getFont$$$(null, -1, 36, label28.getFont());
        if (label28Font != null) label28.setFont(label28Font);
        label28.setText("P: ");
        PIDPanel.add(label28, new com.intellij.uiDesigner.core.GridConstraints(0, 1, 21, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label29 = new JLabel();
        Font label29Font = this.$$$getFont$$$(null, -1, 36, label29.getFont());
        if (label29Font != null) label29.setFont(label29Font);
        label29.setText("I: ");
        PIDPanel.add(label29, new com.intellij.uiDesigner.core.GridConstraints(0, 3, 21, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label30 = new JLabel();
        Font label30Font = this.$$$getFont$$$(null, -1, 36, label30.getFont());
        if (label30Font != null) label30.setFont(label30Font);
        label30.setText("D: ");
        PIDPanel.add(label30, new com.intellij.uiDesigner.core.GridConstraints(0, 5, 21, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label31 = new JLabel();
        Font label31Font = this.$$$getFont$$$(null, Font.BOLD, 36, label31.getFont());
        if (label31Font != null) label31.setFont(label31Font);
        label31.setText("Pitch");
        PIDPanel.add(label31, new com.intellij.uiDesigner.core.GridConstraints(21, 0, 8, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label32 = new JLabel();
        Font label32Font = this.$$$getFont$$$(null, Font.BOLD, 36, label32.getFont());
        if (label32Font != null) label32.setFont(label32Font);
        label32.setText("Roll");
        PIDPanel.add(label32, new com.intellij.uiDesigner.core.GridConstraints(31, 0, 10, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label33 = new JLabel();
        Font label33Font = this.$$$getFont$$$(null, -1, 36, label33.getFont());
        if (label33Font != null) label33.setFont(label33Font);
        label33.setText("P: ");
        PIDPanel.add(label33, new com.intellij.uiDesigner.core.GridConstraints(21, 1, 9, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label34 = new JLabel();
        Font label34Font = this.$$$getFont$$$(null, -1, 36, label34.getFont());
        if (label34Font != null) label34.setFont(label34Font);
        label34.setText("P: ");
        PIDPanel.add(label34, new com.intellij.uiDesigner.core.GridConstraints(31, 1, 13, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label35 = new JLabel();
        Font label35Font = this.$$$getFont$$$(null, -1, 36, label35.getFont());
        if (label35Font != null) label35.setFont(label35Font);
        label35.setText("I: ");
        PIDPanel.add(label35, new com.intellij.uiDesigner.core.GridConstraints(21, 3, 9, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label36 = new JLabel();
        Font label36Font = this.$$$getFont$$$(null, -1, 36, label36.getFont());
        if (label36Font != null) label36.setFont(label36Font);
        label36.setText("I: ");
        PIDPanel.add(label36, new com.intellij.uiDesigner.core.GridConstraints(31, 3, 13, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label37 = new JLabel();
        Font label37Font = this.$$$getFont$$$(null, -1, 36, label37.getFont());
        if (label37Font != null) label37.setFont(label37Font);
        label37.setText("D: ");
        PIDPanel.add(label37, new com.intellij.uiDesigner.core.GridConstraints(21, 5, 9, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label38 = new JLabel();
        Font label38Font = this.$$$getFont$$$(null, -1, 36, label38.getFont());
        if (label38Font != null) label38.setFont(label38Font);
        label38.setText("D: ");
        PIDPanel.add(label38, new com.intellij.uiDesigner.core.GridConstraints(31, 5, 13, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        applyChangesButton = new JButton();
        Font applyChangesButtonFont = this.$$$getFont$$$(null, -1, 36, applyChangesButton.getFont());
        if (applyChangesButtonFont != null) applyChangesButton.setFont(applyChangesButtonFont);
        applyChangesButton.setText("Apply \nChanges");
        PIDPanel.add(applyChangesButton, new com.intellij.uiDesigner.core.GridConstraints(21, 7, 9, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, 1, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, new Dimension(-1, 100), 0, false));
        PitchP = new JTextField();
        PIDPanel.add(PitchP, new com.intellij.uiDesigner.core.GridConstraints(28, 2, 2, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, new Dimension(150, -1), null, 0, false));
        PitchI = new JTextField();
        PIDPanel.add(PitchI, new com.intellij.uiDesigner.core.GridConstraints(27, 4, 3, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, new Dimension(150, -1), null, 0, false));
        PitchD = new JTextField();
        PIDPanel.add(PitchD, new com.intellij.uiDesigner.core.GridConstraints(26, 6, 4, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, new Dimension(150, -1), null, 0, false));
        RollP = new JTextField();
        RollP.setText("");
        RollP.setVisible(true);
        PIDPanel.add(RollP, new com.intellij.uiDesigner.core.GridConstraints(38, 2, 6, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, new Dimension(150, -1), null, 0, false));
        RollI = new JTextField();
        RollI.setText("");
        PIDPanel.add(RollI, new com.intellij.uiDesigner.core.GridConstraints(37, 4, 7, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, new Dimension(150, -1), null, 0, false));
        RollD = new JTextField();
        RollD.setText("");
        PIDPanel.add(RollD, new com.intellij.uiDesigner.core.GridConstraints(35, 6, 9, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, new Dimension(150, -1), null, 0, false));
        YPR_Picture = new JLabel();
        YPR_Picture.setIcon(new ImageIcon(getClass().getResource("/ypr.png")));
        YPR_Picture.setText("");
        YPR_Picture.setVisible(true);
        PIDPanel.add(YPR_Picture, new com.intellij.uiDesigner.core.GridConstraints(0, 7, 21, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label39 = new JLabel();
        Font label39Font = this.$$$getFont$$$(null, Font.BOLD, 20, label39.getFont());
        if (label39Font != null) label39.setFont(label39Font);
        label39.setText("Current Motor Values");
        PIDPanel.add(label39, new com.intellij.uiDesigner.core.GridConstraints(31, 7, 8, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        M1 = new JLabel();
        Font M1Font = this.$$$getFont$$$(null, -1, 20, M1.getFont());
        if (M1Font != null) M1.setFont(M1Font);
        M1.setText("M1: 0.00");
        PIDPanel.add(M1, new com.intellij.uiDesigner.core.GridConstraints(39, 7, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        M0 = new JLabel();
        Font M0Font = this.$$$getFont$$$(null, -1, 20, M0.getFont());
        if (M0Font != null) M0.setFont(M0Font);
        M0.setText("M0: 0.00");
        PIDPanel.add(M0, new com.intellij.uiDesigner.core.GridConstraints(39, 8, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        M2 = new JLabel();
        Font M2Font = this.$$$getFont$$$(null, -1, 20, M2.getFont());
        if (M2Font != null) M2.setFont(M2Font);
        M2.setText("M2: 0.00");
        PIDPanel.add(M2, new com.intellij.uiDesigner.core.GridConstraints(40, 7, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        M3 = new JLabel();
        Font M3Font = this.$$$getFont$$$(null, -1, 20, M3.getFont());
        if (M3Font != null) M3.setFont(M3Font);
        M3.setText("M3: 0.00");
        PIDPanel.add(M3, new com.intellij.uiDesigner.core.GridConstraints(40, 8, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        Font CurrentYawDFont = this.$$$getFont$$$(null, -1, 26, CurrentYawD.getFont());
        if (CurrentYawDFont != null) CurrentYawD.setFont(CurrentYawDFont);
        CurrentYawD.setText("0.0");
        PIDPanel.add(CurrentYawD, new com.intellij.uiDesigner.core.GridConstraints(2, 6, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        CurrentYawI = new JLabel();
        Font CurrentYawIFont = this.$$$getFont$$$(null, -1, 26, CurrentYawI.getFont());
        if (CurrentYawIFont != null) CurrentYawI.setFont(CurrentYawIFont);
        CurrentYawI.setText("0.0");
        PIDPanel.add(CurrentYawI, new com.intellij.uiDesigner.core.GridConstraints(2, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        Font CurrentPitchPFont = this.$$$getFont$$$(null, -1, 26, CurrentPitchP.getFont());
        if (CurrentPitchPFont != null) CurrentPitchP.setFont(CurrentPitchPFont);
        CurrentPitchP.setText("0.0");
        PIDPanel.add(CurrentPitchP, new com.intellij.uiDesigner.core.GridConstraints(21, 2, 7, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        Font CurrentPitchIFont = this.$$$getFont$$$(null, -1, 26, CurrentPitchI.getFont());
        if (CurrentPitchIFont != null) CurrentPitchI.setFont(CurrentPitchIFont);
        CurrentPitchI.setText("0.0");
        PIDPanel.add(CurrentPitchI, new com.intellij.uiDesigner.core.GridConstraints(24, 4, 3, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        Font CurrentPitchDFont = this.$$$getFont$$$(null, -1, 26, CurrentPitchD.getFont());
        if (CurrentPitchDFont != null) CurrentPitchD.setFont(CurrentPitchDFont);
        CurrentPitchD.setText("0.0");
        PIDPanel.add(CurrentPitchD, new com.intellij.uiDesigner.core.GridConstraints(25, 6, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        Font CurrentRollPFont = this.$$$getFont$$$(null, -1, 26, CurrentRollP.getFont());
        if (CurrentRollPFont != null) CurrentRollP.setFont(CurrentRollPFont);
        CurrentRollP.setText("0.0");
        PIDPanel.add(CurrentRollP, new com.intellij.uiDesigner.core.GridConstraints(31, 2, 7, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        Font CurrentRollDFont = this.$$$getFont$$$(null, -1, 26, CurrentRollD.getFont());
        if (CurrentRollDFont != null) CurrentRollD.setFont(CurrentRollDFont);
        CurrentRollD.setText("0.0");
        PIDPanel.add(CurrentRollD, new com.intellij.uiDesigner.core.GridConstraints(34, 6, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        Font CurrentRollIFont = this.$$$getFont$$$(null, -1, 26, CurrentRollI.getFont());
        if (CurrentRollIFont != null) CurrentRollI.setFont(CurrentRollIFont);
        CurrentRollI.setText("0.0");
        PIDPanel.add(CurrentRollI, new com.intellij.uiDesigner.core.GridConstraints(34, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        Font CurrentYawPFont = this.$$$getFont$$$(null, -1, 26, CurrentYawP.getFont());
        if (CurrentYawPFont != null) CurrentYawP.setFont(CurrentYawPFont);
        CurrentYawP.setText("0.0");
        PIDPanel.add(CurrentYawP, new com.intellij.uiDesigner.core.GridConstraints(2, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        currentYaw = new JLabel();
        Font currentYawFont = this.$$$getFont$$$(null, -1, 24, currentYaw.getFont());
        if (currentYawFont != null) currentYaw.setFont(currentYawFont);
        currentYaw.setText("0");
        PIDPanel.add(currentYaw, new com.intellij.uiDesigner.core.GridConstraints(20, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        currentPitch = new JLabel();
        Font currentPitchFont = this.$$$getFont$$$(null, -1, 24, currentPitch.getFont());
        if (currentPitchFont != null) currentPitch.setFont(currentPitchFont);
        currentPitch.setText("0");
        PIDPanel.add(currentPitch, new com.intellij.uiDesigner.core.GridConstraints(30, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        currentRoll = new JLabel();
        Font currentRollFont = this.$$$getFont$$$(null, -1, 24, currentRoll.getFont());
        if (currentRollFont != null) currentRoll.setFont(currentRollFont);
        currentRoll.setText("0");
        PIDPanel.add(currentRoll, new com.intellij.uiDesigner.core.GridConstraints(41, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        YawP = new JTextField();
        PIDPanel.add(YawP, new com.intellij.uiDesigner.core.GridConstraints(3, 2, 16, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, new Dimension(150, -1), null, 0, false));
        YawI = new JTextField();
        PIDPanel.add(YawI, new com.intellij.uiDesigner.core.GridConstraints(5, 4, 13, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, new Dimension(150, -1), null, 0, false));
        YawD = new JTextField();
        PIDPanel.add(YawD, new com.intellij.uiDesigner.core.GridConstraints(6, 6, 9, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, new Dimension(150, -1), null, 0, false));
        GraphPanel = new JPanel();
        GraphPanel.setLayout(new com.intellij.uiDesigner.core.GridLayoutManager(1, 1, new Insets(0, 0, 0, 0), -1, -1));
        PIDPanel.add(GraphPanel, new com.intellij.uiDesigner.core.GridConstraints(44, 0, 1, 9, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        Motors = new JPanel();
        Motors.setLayout(new com.intellij.uiDesigner.core.GridLayoutManager(16, 9, new Insets(0, 0, 0, 0), -1, -1));
        TabPane.addTab("Motors", Motors);
        MotorController0.setValue(0);
        Motors.add(MotorController0, new com.intellij.uiDesigner.core.GridConstraints(1, 0, 1, 9, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final com.intellij.uiDesigner.core.Spacer spacer4 = new com.intellij.uiDesigner.core.Spacer();
        Motors.add(spacer4, new com.intellij.uiDesigner.core.GridConstraints(15, 0, 1, 9, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_VERTICAL, 1, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, null, null, null, 0, false));
        MotorController1.setValue(0);
        Motors.add(MotorController1, new com.intellij.uiDesigner.core.GridConstraints(5, 0, 1, 9, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        MotorController2.setValue(0);
        Motors.add(MotorController2, new com.intellij.uiDesigner.core.GridConstraints(9, 0, 1, 9, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        MotorController3.setValue(0);
        Motors.add(MotorController3, new com.intellij.uiDesigner.core.GridConstraints(13, 0, 1, 9, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label40 = new JLabel();
        Font label40Font = this.$$$getFont$$$(null, Font.BOLD, 24, label40.getFont());
        if (label40Font != null) label40.setFont(label40Font);
        label40.setText("Motor 0 (Front Right)");
        Motors.add(label40, new com.intellij.uiDesigner.core.GridConstraints(0, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label41 = new JLabel();
        Font label41Font = this.$$$getFont$$$(null, Font.BOLD, 24, label41.getFont());
        if (label41Font != null) label41.setFont(label41Font);
        label41.setText("Motor 1 (Front Left)");
        Motors.add(label41, new com.intellij.uiDesigner.core.GridConstraints(4, 3, 1, 3, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label42 = new JLabel();
        Font label42Font = this.$$$getFont$$$(null, Font.BOLD, 24, label42.getFont());
        if (label42Font != null) label42.setFont(label42Font);
        label42.setText("Motor 2 (Back Left)");
        Motors.add(label42, new com.intellij.uiDesigner.core.GridConstraints(8, 2, 1, 5, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label43 = new JLabel();
        Font label43Font = this.$$$getFont$$$(null, Font.BOLD, 24, label43.getFont());
        if (label43Font != null) label43.setFont(label43Font);
        label43.setText("Motor 3 (Back Right)");
        Motors.add(label43, new com.intellij.uiDesigner.core.GridConstraints(12, 1, 1, 7, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label44 = new JLabel();
        label44.setText("0");
        Motors.add(label44, new com.intellij.uiDesigner.core.GridConstraints(0, 0, 1, 4, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label45 = new JLabel();
        label45.setText("100");
        Motors.add(label45, new com.intellij.uiDesigner.core.GridConstraints(0, 5, 1, 4, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_EAST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label46 = new JLabel();
        label46.setText("0");
        Motors.add(label46, new com.intellij.uiDesigner.core.GridConstraints(4, 0, 1, 3, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label47 = new JLabel();
        label47.setText("0");
        Motors.add(label47, new com.intellij.uiDesigner.core.GridConstraints(8, 0, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label48 = new JLabel();
        label48.setText("0");
        Motors.add(label48, new com.intellij.uiDesigner.core.GridConstraints(12, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label49 = new JLabel();
        label49.setText("100");
        Motors.add(label49, new com.intellij.uiDesigner.core.GridConstraints(4, 6, 1, 3, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_EAST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label50 = new JLabel();
        label50.setText("100");
        Motors.add(label50, new com.intellij.uiDesigner.core.GridConstraints(8, 7, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_EAST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label51 = new JLabel();
        label51.setText("100");
        Motors.add(label51, new com.intellij.uiDesigner.core.GridConstraints(12, 8, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_EAST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        Font Speed0Font = this.$$$getFont$$$(null, Font.BOLD, 24, Speed0.getFont());
        if (Speed0Font != null) Speed0.setFont(Speed0Font);
        Speed0.setText("0");
        Motors.add(Speed0, new com.intellij.uiDesigner.core.GridConstraints(2, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        Font Speed1Font = this.$$$getFont$$$(null, Font.BOLD, 24, Speed1.getFont());
        if (Speed1Font != null) Speed1.setFont(Speed1Font);
        Speed1.setText("0");
        Motors.add(Speed1, new com.intellij.uiDesigner.core.GridConstraints(6, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        Font Speed2Font = this.$$$getFont$$$(null, Font.BOLD, 24, Speed2.getFont());
        if (Speed2Font != null) Speed2.setFont(Speed2Font);
        Speed2.setText("0");
        Motors.add(Speed2, new com.intellij.uiDesigner.core.GridConstraints(10, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        Font Speed3Font = this.$$$getFont$$$(null, Font.BOLD, 24, Speed3.getFont());
        if (Speed3Font != null) Speed3.setFont(Speed3Font);
        Speed3.setText("0");
        Motors.add(Speed3, new com.intellij.uiDesigner.core.GridConstraints(14, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label52 = new JLabel();
        label52.setText("");
        Motors.add(label52, new com.intellij.uiDesigner.core.GridConstraints(3, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label53 = new JLabel();
        label53.setText("");
        Motors.add(label53, new com.intellij.uiDesigner.core.GridConstraints(7, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label54 = new JLabel();
        label54.setText("");
        Motors.add(label54, new com.intellij.uiDesigner.core.GridConstraints(11, 4, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        DriverPanel = new JPanel();
        DriverPanel.setLayout(new com.intellij.uiDesigner.core.GridLayoutManager(9, 5, new Insets(0, 0, 0, 0), -1, -1));
        panelMain.add(DriverPanel, new com.intellij.uiDesigner.core.GridConstraints(0, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        DriverPanel.setBorder(BorderFactory.createTitledBorder(" "));
        ElapsedTimeLabel = new JLabel();
        Font ElapsedTimeLabelFont = this.$$$getFont$$$(null, -1, 28, ElapsedTimeLabel.getFont());
        if (ElapsedTimeLabelFont != null) ElapsedTimeLabel.setFont(ElapsedTimeLabelFont);
        ElapsedTimeLabel.setText("Elapsed Time: 00:00:000");
        DriverPanel.add(ElapsedTimeLabel, new com.intellij.uiDesigner.core.GridConstraints(1, 0, 1, 5, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        Enable_Status = new JButton();
        Enable_Status.setBackground(new Color(-11842741));
        Enable_Status.setFocusPainted(false);
        Enable_Status.setFocusable(false);
        Font Enable_StatusFont = this.$$$getFont$$$("Arial", Font.BOLD, 96, Enable_Status.getFont());
        if (Enable_StatusFont != null) Enable_Status.setFont(Enable_StatusFont);
        Enable_Status.setForeground(new Color(-65529));
        Enable_Status.setMargin(new Insets(2, 14, 2, 14));
        Enable_Status.setText("Disabled");
        Enable_Status.setVerifyInputWhenFocusTarget(true);
        DriverPanel.add(Enable_Status, new com.intellij.uiDesigner.core.GridConstraints(0, 0, 1, 5, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        BatteryVoltage = new JLabel();
        BatteryVoltage.setEnabled(true);
        Font BatteryVoltageFont = this.$$$getFont$$$(null, -1, 28, BatteryVoltage.getFont());
        if (BatteryVoltageFont != null) BatteryVoltage.setFont(BatteryVoltageFont);
        BatteryVoltage.setText("Battery Voltage: 0.00 V");
        DriverPanel.add(BatteryVoltage, new com.intellij.uiDesigner.core.GridConstraints(2, 0, 1, 5, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label55 = new JLabel();
        label55.setText("Select Antenna COM Port");
        DriverPanel.add(label55, new com.intellij.uiDesigner.core.GridConstraints(3, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        COMCombo.setLightWeightPopupEnabled(true);
        DriverPanel.add(COMCombo, new com.intellij.uiDesigner.core.GridConstraints(4, 0, 1, 4, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        ConsoleScrollPane = new JScrollPane();
        DriverPanel.add(ConsoleScrollPane, new com.intellij.uiDesigner.core.GridConstraints(8, 0, 1, 4, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, null, null, null, 0, false));
        Console = new JTextPane();
        Console.setEditable(false);
        ConsoleScrollPane.setViewportView(Console);
        DriverPanel.add(ControllerCombo, new com.intellij.uiDesigner.core.GridConstraints(6, 0, 1, 4, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        final JLabel label56 = new JLabel();
        label56.setText("Select Controller");
        DriverPanel.add(label56, new com.intellij.uiDesigner.core.GridConstraints(5, 0, 1, 3, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_WEST, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        mpuZeroButton = new JButton();
        Font mpuZeroButtonFont = this.$$$getFont$$$(null, Font.BOLD, 18, mpuZeroButton.getFont());
        if (mpuZeroButtonFont != null) mpuZeroButton.setFont(mpuZeroButtonFont);
        mpuZeroButton.setText("Calibrate Gyroscope and Accelerometer");
        DriverPanel.add(mpuZeroButton, new com.intellij.uiDesigner.core.GridConstraints(7, 0, 1, 5, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        resetButton = new JButton();
        resetButton.setText("Reset");
        DriverPanel.add(resetButton, new com.intellij.uiDesigner.core.GridConstraints(3, 2, 1, 2, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        controllerWarningPanel = new JPanel();
        controllerWarningPanel.setLayout(new com.intellij.uiDesigner.core.GridLayoutManager(1, 1, new Insets(0, 0, 0, 0), -1, -1));
        DriverPanel.add(controllerWarningPanel, new com.intellij.uiDesigner.core.GridConstraints(5, 3, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, null, null, null, 0, false));
        final com.intellij.uiDesigner.core.Spacer spacer5 = new com.intellij.uiDesigner.core.Spacer();
        DriverPanel.add(spacer5, new com.intellij.uiDesigner.core.GridConstraints(3, 1, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, 1, null, null, null, 0, false));
    }

    /**
     * @noinspection ALL
     */
    private Font $$$getFont$$$(String fontName, int style, int size, Font currentFont)
    {
        if (currentFont == null) return null;
        String resultName;
        if (fontName == null)
        {
            resultName = currentFont.getName();
        } else
        {
            Font testFont = new Font(fontName, Font.PLAIN, 10);
            if (testFont.canDisplay('a') && testFont.canDisplay('1'))
            {
                resultName = fontName;
            } else
            {
                resultName = currentFont.getName();
            }
        }
        return new Font(resultName, style >= 0 ? style : currentFont.getStyle(), size >= 0 ? size : currentFont.getSize());
    }

    /**
     * @noinspection ALL
     */
    public JComponent $$$getRootComponent$$$()
    {
        return panelMain;
    }

    /*
    TODO: fix crash from adding/removing a USB serial comm port, change selected port to global port so it closes when you choose another one
     */
}

abstract class TimeLimitedCodeBlock {

    public static void runWithTimeout(final Runnable runnable, long timeout, TimeUnit timeUnit) throws Exception {
        runWithTimeout(new Callable<Object>() {
            @Override
            public Object call() throws Exception {
                runnable.run();
                return null;
            }
        }, timeout, timeUnit);
    }

    public static <T> T runWithTimeout(Callable<T> callable, long timeout, TimeUnit timeUnit) throws Exception {
        final ExecutorService executor = Executors.newSingleThreadExecutor();
        final Future<T> future = executor.submit(callable);
        executor.shutdown(); // This does not cancel the already-scheduled task.
        try {
            return future.get(timeout, timeUnit);
        }
        catch (TimeoutException e) {
            //remove this if you do not want to cancel the job in progress
            //or set the argument to 'false' if you do not want to interrupt the thread
            future.cancel(true);
            throw e;
        }
        catch (ExecutionException e) {
            //unwrap the root cause
            Throwable t = e.getCause();
            if (t instanceof Error) {
                throw (Error) t;
            } else if (t instanceof Exception) {
                throw (Exception) t;
            } else {
                throw new IllegalStateException(t);
            }
        }
    }

    public abstract void codeBlock();
}
