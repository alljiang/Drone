import com.fazecast.jSerialComm.SerialPort;

import java.io.PrintWriter;
import java.util.Scanner;

public class SerialPortTest
{

    public static void main(String[] args)
    {
        for(SerialPort sp : SerialPort.getCommPorts()) System.out.println(sp.getSystemPortName());
        SerialPort chosenPort = SerialPort.getCommPorts()[0];
        if(chosenPort.openPort())
        {
            System.out.println("Opened Port");
            chosenPort.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
            chosenPort.setBaudRate(115200);
            chosenPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, 0, 0);
            Scanner data = new Scanner(chosenPort.getInputStream());
            PrintWriter output = new PrintWriter(chosenPort.getOutputStream());
            while(true)
            {
                System.out.println("in loop " + data.hasNextLine() + " " + data.hasNext());
                if(!data.hasNextLine())
                {
                    data = new Scanner(chosenPort.getInputStream());
                    continue;
                }
                while(data.hasNextLine())
                    System.out.println(data.nextLine());
//                output.println("2:5:0:0:0");
//                output.flush();
                try
                {
                    Thread.sleep(1000);
                } catch (Exception e){}
            }
        }
        else System.out.println("NO");
    }

}

/*
Serial Port:

SerialPort chosenPort;
SerialPort[] portNames = SerialPort.getCommPorts();
chosenPort = SerialPort.getCommPort(portNames[<PORT>].toString());
chosenPort.setComPortTimeouts(SerialPort.TIMEOUT_SCANNER, 0, 0);
if(chosenPort.openport()) {
    PrintWriter output = new PrintWriter(chosenPort.getOutputStream());
    output.print("TEXT");
    output.flush();
} else {
    chosePort.closePort();
}

 */