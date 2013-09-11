package Main;

import Ros.RosWrapper;

import javax.swing.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

/**
 * Created by IntelliJ IDEA.
 * User: homer
 * Date: Oct 18, 2011
 * Time: 10:42:05 AM
 * To change this template use File | Settings | File Templates.
 */
public class Test {
    public static void main(String[] args){
        JFrame frame=new JFrame();
        final boolean[] downkeys=new boolean[5];

        KeyListener kl=new KeyListener(){
            public void keyTyped(KeyEvent e) {
                /*try{
                    switch(e.getKeyCode()){
                        case KeyEvent.VK_UP:
                            dataout.write(RosWrapper.CMDS_FORWARD);
                            dataout.flush();
                            //ros.sendCommand(RosWrapper.CMD_FORWARD);
                            break;
                        case KeyEvent.VK_LEFT:
                            dataout.write(RosWrapper.CMDS_LEFT);
                            dataout.flush();
                            //ros.sendCommand(RosWrapper.CMD_LEFT);
                            break;
                        case KeyEvent.VK_RIGHT:
                            dataout.write(RosWrapper.CMDS_RIGHT);
                            dataout.flush();
                            //ros.sendCommand(RosWrapper.CMD_RIGHT);
                            break;
                        case KeyEvent.VK_DOWN:
                            dataout.write(RosWrapper.CMDS_BACK);
                            dataout.flush();
                            //ros.sendCommand(RosWrapper.CMD_BACK);
                            break;
                        case KeyEvent.VK_SPACE:
                            dataout.write(RosWrapper.CMDS_STOP);
                            dataout.flush();
                            //ros.sendCommand(RosWrapper.CMD_STOP);
                            break;
                    }
                }catch(Exception ex){
                    ex.printStackTrace();
                }*/
            }
            public void keyPressed(KeyEvent e) {
                switch(e.getKeyCode()){
                    case KeyEvent.VK_UP:
                        downkeys[0]=true;
                        break;
                    case KeyEvent.VK_LEFT:
                        downkeys[1]=true;
                        break;
                    case KeyEvent.VK_RIGHT:
                        downkeys[2]=true;
                        break;
                    case KeyEvent.VK_DOWN:
                        downkeys[3]=true;
                        break;
                    case KeyEvent.VK_SPACE:
                        downkeys[4]=true;
                        break;
                }
                try{
                    if (downkeys[4]){
                        System.out.println("Stop");
                        return;
                    }
                    if (downkeys[3]){
                        System.out.println("Back");
                        return;
                    }
                    if (downkeys[0]){
                        if (downkeys[1]){
                            System.out.println("Forward left");
                            return;
                        }
                        if (downkeys[2]){
                            System.out.println("Forward right");
                            return;
                        }
                        System.out.println("Forward");
                        return;
                    }
                    if (downkeys[1]){
                        System.out.println("Left");
                        return;
                    }
                    if (downkeys[2]){
                        System.out.println("Right");
                        return;

                    }

                }catch(Exception ex){
                    ex.printStackTrace();
                }
            }
            public void keyReleased(KeyEvent e) {
               switch(e.getKeyCode()){
                    case KeyEvent.VK_UP:
                        downkeys[0]=false;
                        break;
                    case KeyEvent.VK_LEFT:
                        downkeys[1]=false;
                        break;
                    case KeyEvent.VK_RIGHT:
                        downkeys[2]=false;
                        break;
                    case KeyEvent.VK_DOWN:
                        downkeys[3]=false;
                        break;
                    case KeyEvent.VK_SPACE:
                        downkeys[4]=false;
                        break;
                }
            }
        };
        ImagePanel panel=new ImagePanel(true,kl);
        frame.add(panel);
        frame.addKeyListener(kl);
	    frame.setSize(320,240);
        frame.setVisible(true);
    }
}
