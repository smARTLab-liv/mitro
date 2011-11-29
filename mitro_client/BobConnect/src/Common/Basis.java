/*
Copyright (C) 2011

This file is part of JHuffman
written by Max Bügler
http://www.maxbuegler.eu/

JHuffman is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 2, or (at your option) any
later version.

JHuffman is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

package Common;

/**
 * Collection of conversion methods byte/int and boolean/byte
 */
public class Basis {

    /**
     * Converts a positive int to 3 bytes.
     * @param v small int value
     * @return  byte[3] array
     */
    public static byte[] intToBytes(int v){

        //Create output array
        byte[] out=new byte[3];

        //256^2 byte
        out[0]=(byte)((v/(256*256))-128);
        v=v%(256*256);

        //256 byte
        out[1]=(byte)((v/256)-128);
        v=v%256;

        //1 byte
        out[2]=(byte)(v-128);
        return out;

    }

    /**
     * Converts 3 bytes to an int
     * @param v byte[3] array
     * @return int
     */
    public static int bytesToInt(byte[] v){
        return (256*256*(v[0]+128))+(256*(v[1]+128))+v[2]+128;
    }

    /**
     * Converts bits to byte (max 8)
      * @param b max 8 bits
     * @return byte value
     */
    public static byte booleansToByte(boolean[] b){
         byte res=-128;
         for (int x=0;x<b.length&&x<8;x++){
             if (b[x])res+=Math.pow(2,x);
         }
         return res;
     }

    /**
     * Converts byte to 8 bits
     * @param b byte
     * @return boolean[]
     */
    public static boolean[] byteToBooleans(byte b){
         boolean[] out=new boolean[8];
         int v=b+128;
         for (int x=0;x<8;x++){
             if (v%Math.pow(2,x+1)>0){
                 v-=v%Math.pow(2,x+1);
                 out[x]=true;
             }
             else{
                 out[x]=false;
             }
         }
         return out;
    }


    /**
     * Converts int to n bits
     * @param v int
     * @return boolean[]
     */
    public static boolean[] intToBooleans(int v, int bits){
         boolean[] out=new boolean[bits];
         for (int x=0;x<bits;x++){
             if (v%Math.pow(2,x+1)>0){
                 v-=v%Math.pow(2,x+1);
                 out[x]=true;
             }
             else{
                 out[x]=false;
             }
         }
         return out;
    }
    /**
     * Converts bits to positive int
      * @param b
     * @return byte value
     */
    public static int booleansToInt(boolean[] b){
         int res=0;
         for (int x=0;x<b.length;x++){
             if (b[x])res+=Math.pow(2,x);
         }
         return res;
     }


}
