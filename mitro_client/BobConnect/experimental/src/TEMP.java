import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;

/**
 * Created by IntelliJ IDEA.
 * User: homer
 * Date: Oct 24, 2011
 * Time: 11:13:45 AM
 * To change this template use File | Settings | File Templates.
 */
public class TEMP {
    public static void main(String[] args) throws Exception{
        for (int x=0;x<=20;x++){
            String name=x+"";
            if (x<10)name="0"+x;
            BufferedImage img=ImageIO.read(new File("/home/homer/Code/Java/BobConnectTemp/gui4/bobgui00"+name+".png"));
            ImageIO.write(img.getSubimage(906,720,108,108),"png",new File("/home/homer/Code/Java/BobConnectTemp/gui4/cutbobgui00"+name+".png"));
        }
    }
}
