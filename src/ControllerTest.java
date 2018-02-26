import org.lwjgl.LWJGLException;
import org.lwjgl.input.Controller;
import org.lwjgl.input.Controllers;

public class ControllerTest
{

    static Controller controller;

    public static void main(String[] args)
    {
        try
        {
            Controllers.create();
        } catch (LWJGLException e)
        {
            e.printStackTrace();
        }
        Controllers.poll();

        for (int i = 0; i < Controllers.getControllerCount(); i++)
        {
            controller = Controllers.getController(i);
            System.out.println(controller.getName());
        }
        controller = Controllers.getController(9);
        boolean press;
        while(true)
        {
            controller.poll();
            press = controller.isButtonPressed(12);
            String d = controller.getAxisName(0);
            System.out.println(press);
        }
    }

}
