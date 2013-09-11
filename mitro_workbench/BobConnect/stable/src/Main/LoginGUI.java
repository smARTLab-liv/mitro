package Main;

/*
Copyright (C) 2011

This file is part of BobConnect
written by Max Bügler
http://www.maxbuegler.eu/

BobConnect is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 2, or (at your option) any
later version.

BobConnect is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.image.BufferedImage;
import java.io.File;

public class LoginGUI extends JPanel {
    private BufferedImage back,button;
    private JTextField login;
    private JPasswordField password;
    private JLabel status;
    private BackPanel background;
    private ButtonPanel buttonpanel;
    private final int[][] LOCATIONS=new int[][]{{374,171,213,29},{374,262,213,29},{357,333,250,66},{88,419,420,20}};
    private boolean buttonhover;

    private boolean done;


    public LoginGUI() throws Exception{
        super(null);
        back= ImageIO.read(new File("logingui.png"));
        button= ImageIO.read(new File("loginbutton.png"));
        this.buttonhover=false;
        this.done=false;
        background=new BackPanel();
        background.setLocation(0,0);
        background.setSize(640,480);
        buttonpanel=new ButtonPanel();
        buttonpanel.setLocation(LOCATIONS[3][0],LOCATIONS[3][1]);
        buttonpanel.setSize(LOCATIONS[3][2],LOCATIONS[3][3]);
        buttonpanel.setOpaque(false);
        login=new JTextField();
        login.setBorder(BorderFactory.createEmptyBorder());
        login.setLocation(LOCATIONS[0][0],LOCATIONS[0][1]);
        login.setSize(LOCATIONS[0][2],LOCATIONS[0][3]);
        password=new JPasswordField();
        password.setBorder(BorderFactory.createEmptyBorder());
        password.setLocation(LOCATIONS[1][0],LOCATIONS[1][1]);
        password.setSize(LOCATIONS[1][2],LOCATIONS[1][3]);

        status=new JLabel("",JLabel.CENTER);
        status.setLocation(LOCATIONS[3][0],LOCATIONS[3][1]);
        status.setSize(LOCATIONS[3][2],LOCATIONS[3][3]);


        Listener L=new Listener();
        addMouseMotionListener(L);
        addMouseListener(L);
        //add(background);
        add(login);
        add(password);
        add(status);
        //add(buttonpanel);
    }

    public void update(Graphics g){
        paintComponent(g);
    }
    
    public void paintComponent(Graphics g){
        g.drawImage(back,0,0,null);
        if (buttonhover){
            g.drawImage(button,LOCATIONS[2][0],LOCATIONS[2][1],null);
        }
        /*login.paint(g);
        password.paint(g);*/        
    }

    public String getUsername(){
        return login.getText();
    }

    public String getPassword(){
        return new String(password.getPassword());
    }

    public boolean isDone(){
        return done;
    }

    public void reset(){
        done=false;
        //status.setText("");
        password.setText("");

    }

    public void setStatus(String s){
        status.setText(s);
    }

    class Listener implements MouseListener, MouseMotionListener {
        public void mouseClicked(MouseEvent e) {
            if (e.getX()>LOCATIONS[2][0]&&e.getX()<LOCATIONS[2][0]+LOCATIONS[2][2]
                    &&e.getY()>LOCATIONS[2][1]&&e.getY()<LOCATIONS[2][1]+LOCATIONS[2][3]){
                done=true;
            }
        }

        public void mousePressed(MouseEvent e) {

        }

        public void mouseReleased(MouseEvent e) {

        }

        public void mouseEntered(MouseEvent e) {

        }

        public void mouseExited(MouseEvent e) {

        }

        public void mouseDragged(MouseEvent e) {

        }

        public void mouseMoved(MouseEvent e) {
            if (e.getX()>LOCATIONS[2][0]&&e.getX()<LOCATIONS[2][0]+LOCATIONS[2][2]
                    &&e.getY()>LOCATIONS[2][1]&&e.getY()<LOCATIONS[2][1]+LOCATIONS[2][3]){
                if (!buttonhover){
                    buttonhover=true;
                    repaint();
                }
            }
            else if (buttonhover){
                buttonhover=false;
                repaint();
            }
        }
    }

    class BackPanel extends JPanel{
        public void paintComponent(Graphics g){
            g.drawImage(back,0,0,null);
        }
    }

    class ButtonPanel extends JPanel{
        public void paintComponent(Graphics g){
            g.drawImage(button,LOCATIONS[2][0],LOCATIONS[2][1],null);
        }
    }

}
