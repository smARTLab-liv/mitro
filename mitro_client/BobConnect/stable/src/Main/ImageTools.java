/*
Copyright (C) 2011

This file is part of JLZ77
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

package Main;

import java.awt.*;
import java.awt.image.BufferedImage;

public class ImageTools {
    public static double getDifference(BufferedImage img, BufferedImage prev){
        if (prev==null)return 10000;
        long diff=0;
        for (int x=0;x<img.getWidth();x+=5){
            for (int y=0;y<img.getHeight();y+=5){
                Color c1=new Color(img.getRGB(x, y));
                Color c2=new Color(prev.getRGB(x, y));
                diff+=Math.abs(c1.getRed()-c2.getRed())+Math.abs(c1.getGreen()-c2.getGreen())+Math.abs(c1.getBlue()-c2.getBlue());
            }
        }
        return (double)(25*diff)/(img.getWidth()*img.getHeight());
    }

    public static void cropImage(BufferedImage in, BufferedImage out){
        int xoffset=(in.getWidth()-out.getWidth())/2;
        int yoffset=(in.getHeight()-out.getHeight())/2;
        for (int x=0;x<out.getWidth();x++){
            for (int y=0;y<out.getHeight();y++){
                int ox=x+xoffset;
                int oy=y+yoffset;
                if (ox>=0&&ox<in.getWidth()&&oy>=0&&oy<in.getHeight())
                    out.setRGB(x,y,in.getRGB(ox,oy));
            }
        }
    }

    public static BufferedImage cropImage(BufferedImage in, int width, int height){

        int xoffset=(in.getWidth()-width)/2;
        int yoffset=(in.getHeight()-height)/2;
        return in.getSubimage(xoffset,yoffset,width,height);
    }

    public static BufferedImage copyImage(BufferedImage in){
        BufferedImage out=new BufferedImage(in.getWidth(),in.getHeight(),in.getType());
        Graphics g=out.getGraphics();
        g.drawImage(in,0,0,null);
        return out;
    }

    
    public static void substract(BufferedImage newImage, BufferedImage lastImage,BufferedImage res){
        for (int x=0;x<newImage.getWidth();x++){
            for (int y=0;y<newImage.getHeight();y++){
                Color c1=new Color(newImage.getRGB(x,y));
                Color c2=new Color(lastImage.getRGB(x,y));
                int cr=Math.max(0,Math.min(255,128+((c1.getRed()-c2.getRed())/2)));
                int cg=Math.max(0,Math.min(255,128+((c1.getGreen()-c2.getGreen())/2)));
                int cb=Math.max(0,Math.min(255,128+((c1.getBlue()-c2.getBlue())/2)));
                //if (Math.abs(cr-128)<5)cr=128;
                //if (Math.abs(cg-128)<5)cg=128;
                //if (Math.abs(cb-128)<5)cb=128;
                res.setRGB(x,y,new Color(cr,cg,cb).getRGB());
            }
        }
    }
    public static void add(BufferedImage deltaImage, BufferedImage lastImage,BufferedImage res){

        for (int x=0;x<deltaImage.getWidth();x++){
            for (int y=0;y<deltaImage.getHeight();y++){
                Color c1=new Color(deltaImage.getRGB(x,y));
                Color c2=new Color(lastImage.getRGB(x,y));
                int cr=Math.max(0,Math.min(255,c2.getRed()+((c1.getRed()-128)*2)));
                int cg=Math.max(0,Math.min(255,c2.getGreen()+((c1.getGreen()-128)*2)));
                int cb=Math.max(0,Math.min(255,c2.getBlue()+((c1.getBlue()-128)*2)));
                res.setRGB(x,y,new Color(cr,cg,cb).getRGB());
            }
        }
    }

    public static BufferedImage scaleImageToWidth(BufferedImage img, int width, int type){
        double scale=(double)width/img.getWidth();
        int height=(int)Math.round(img.getHeight()*scale);
        BufferedImage out=new BufferedImage(width,height, type);
        Graphics2D g=(Graphics2D)out.getGraphics();
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
                            RenderingHints.VALUE_INTERPOLATION_BICUBIC);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                            RenderingHints.VALUE_ANTIALIAS_ON);
        g.drawImage(img,0,0,width,height,null);
        return out;
    }

    public static BufferedImage scaleImageToHeight(BufferedImage img, int height, int type){
        double scale=(double)height/img.getHeight();
        int width=(int)Math.round(img.getWidth()*scale);
        BufferedImage out=new BufferedImage(width,height, type);
        Graphics2D g=(Graphics2D)out.getGraphics();
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
                            RenderingHints.VALUE_INTERPOLATION_BICUBIC);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                            RenderingHints.VALUE_ANTIALIAS_ON);

        g.drawImage(img,0,0,width,height,null);
        return out;
    }

    public static BufferedImage scaleImageToRectangle(BufferedImage img, int width, int height, int type){
        double r1=(double)img.getWidth()/img.getHeight();
        double r2=(double)width/height;
        if (r1>r2)
            return scaleImageToWidth(img,width,type);
        return scaleImageToHeight(img,height,type);
    }

    public static BufferedImage scaleImage(BufferedImage img, double scale, int type){
        int width=(int)Math.round(img.getWidth()*scale);
        int height=(int)Math.round(img.getHeight()*scale);
        BufferedImage out=new BufferedImage(width,height, type);
        Graphics2D g=(Graphics2D)out.getGraphics();
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
                            RenderingHints.VALUE_INTERPOLATION_BICUBIC);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                            RenderingHints.VALUE_ANTIALIAS_ON);
        g.drawImage(img,0,0,width,height,null);
        return out;
    }    
}
