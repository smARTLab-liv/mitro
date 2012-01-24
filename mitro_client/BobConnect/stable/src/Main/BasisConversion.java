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

/**
 * Converts between datatypes
 */

class BasisConversion{

    /**
     * Frees a byte value to be used as terminator
     * @param data input data
     * @param b1 byte 1
     * @param b2 byte 2
     * @param free free byte
     * @return byte[] not using free byte
     */
    public static byte[] freeByte(byte[] data, byte b1, byte b2, byte free){
        byte[] temp=new byte[2*data.length];
        int pointer=0;
        for (int x=0;x<data.length;x++){
            if (data[x]==b1){
                temp[pointer++]=b1;
                temp[pointer++]=b1;
            }
            else if (data[x]==b2){
                temp[pointer++]=b1;
                temp[pointer++]=b2;
            }
            else if (data[x]==free){
                temp[pointer++]=b2;
                temp[pointer++]=b1;
            }
            else{
                temp[pointer++]=data[x];
            }
        }
        byte[] out=new byte[pointer];
        for (int x=0;x<pointer;x++){
            out[x]=temp[x];
        }
        return out;
    }


    /**
     * Frees a byte value to be used as terminator
     * @param data input data
     * @param b1 byte 1
     * @param b2 byte 2
     * @param term free byte
     * @return byte[] not using free byte and terminates
     */
    public static byte[] freeByteAndTerminate(byte[] data, byte b1, byte b2, byte term){
        byte[] temp=new byte[2*data.length+3];
        int pointer=0;
        for (int x=0;x<data.length;x++){
            if (data[x]==b1){
                temp[pointer++]=b1;
                temp[pointer++]=b1;
            }
            else if (data[x]==b2){
                temp[pointer++]=b1;
                temp[pointer++]=b2;
            }
            else if (data[x]==term){
                temp[pointer++]=b2;
                temp[pointer++]=b1;
            }
            else{
                temp[pointer++]=data[x];
            }
        }
        temp[pointer++]=term;
        temp[pointer++]=term;
        temp[pointer++]=term;
        byte[] out=new byte[pointer];
        for (int x=0;x<pointer;x++){
            out[x]=temp[x];
        }
        return out;
    }

    /**
     * Restore freed byte
     * @param data input
     * @param b1 byte 1
     * @param b2 byte 2
     * @param free free byte
     * @return restored byte[]
     */
    public static byte[] restoreByte(byte[] data,byte b1, byte b2, byte free){
        byte[] temp=new byte[data.length];
        int pointer=0;
        for (int x=0;x<data.length;x++){
            if (data[x]==b2){
                x++;
                if (x<data.length&&data[x]==b1){
                    temp[pointer++]=free;
                }
            }
            else if (data[x]==b1){
                x++;
                if (x<data.length&&data[x]==b1){
                    temp[pointer++]=b1;
                }
                else if (data[x]==b2){
                    temp[pointer++]=b2;
                }

            }
            else{
                temp[pointer++]=data[x];
            }
        }
        byte[] out=new byte[pointer];
        for (int x=0;x<pointer;x++){
            out[x]=temp[x];
        }
        return out;
    }

    /**
     * Converts string to UTF8
     * @param s input
     * @return byte[]
     */
    public static byte[] stringToUTF8(String s){
        try{
            return s.getBytes("UTF8");
        }
        catch(Exception ex){ex.printStackTrace();}
        return null;
    }

    /**
     * Converts UTF8 to string
     * @param b input
     * @return String
     */
    public static String UTF8toString(byte[] b){
        try{
            return new String(b,"UTF8");
        }
        catch(Exception ex){ex.printStackTrace();}
        return null;
    }

    private static final String HEX="0123456789abcdef";

    public static String convertBytesToHex(byte[] bytes){
        String out="";
        for (int x=0;x<bytes.length;x++){
            byte v=bytes[x];
            out+=HEX.charAt((v+128)/16);
            out+=HEX.charAt((v+128)%16);
        }
        return out;
    }

    public static byte[] convertHexToBytes(String hex){
        byte[] out=new byte[hex.length()/2];
        for (int x=0;x<hex.length()-1;x+=2){
            int res=
                    16*HEX.indexOf(hex.charAt( x ))+
                       HEX.indexOf(hex.charAt(x+1));
            out[x/2]=(byte)(res-128);
        }
        return out;
    }

    /**
     * Converts a positive int to 3 bytes.
     * @param v small int value
     * @return  byte[3] array
     */
    /*public static byte[] intToBytes(int v){

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

    } */

    /**
     * Converts 3 bytes to an int
     * @param data byte[3] array
     * @return int
     */
    /*public static int bytesToInt(byte[] v){
        return (256*256*(v[0]+128))+(256*(v[1]+128))+v[2]+128;
    } */

  public static byte[] intToBytes(int data) {
        return new byte[] {
            //(byte)( data >> 24 ),
            //(byte)( (data << 8) >> 24 ),
            //(byte)( (data << 16) >> 24 ),
            //(byte)( (data << 24) >> 24 )
            (byte)((data >> 24) & 0xff),
            (byte)((data >> 16) & 0xff),
            (byte)((data >> 8) & 0xff),
            (byte)((data >> 0) & 0xff)
        };
    }

    public static int bytesToInt(byte[] data) {
        if (data == null || data.length != 4) return 0x0;
        return (
            (0xff & data[0]) << 24 |
            (0xff & data[1]) << 16 |
            (0xff & data[2]) << 8 |
            (0xff & data[3]) << 0
        );
    }



    public static byte[] longToBytes(long data) {
        return new byte[] {
            (byte)((data >> 56) & 0xff),
            (byte)((data >> 48) & 0xff),
            (byte)((data >> 40) & 0xff),
            (byte)((data >> 32) & 0xff),
            (byte)((data >> 24) & 0xff),
            (byte)((data >> 16) & 0xff),
            (byte)((data >> 8) & 0xff),
            (byte)((data >> 0) & 0xff)
        };
    }

    public static long bytesToLong(byte[] data) {
        if (data == null || data.length != 8) return 0x0;
        return (
            (long)(0xff & data[0]) << 56 |
            (long)(0xff & data[1]) << 48 |
            (long)(0xff & data[2]) << 40 |
            (long)(0xff & data[3]) << 32 |
            (long)(0xff & data[4]) << 24 |
            (long)(0xff & data[5]) << 16 |
            (long)(0xff & data[6]) << 8 |
            (long)(0xff & data[7]) << 0
        );
    }

}
