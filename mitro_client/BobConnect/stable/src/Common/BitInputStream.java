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

package Common;

import java.io.*;

/**
 * Reads binary data from file
 */
public class BitInputStream {
    private InputStream fis; //File input stream
    private boolean[] buffer; //Buffer
    private int bufferindex; //Pointer index for buffer
    private boolean bitsleft;

    /**
     * Construct stream from file
     * @param f file
     * @throws FileNotFoundException
     */
    public BitInputStream(File f) throws FileNotFoundException {
        this.fis=new FileInputStream(f);
        this.bufferindex=8;
        this.buffer=new boolean[8];
        this.bitsleft=true;
    }

    /**
     * Construct stream from inputstream
     * @param fis fileinputstream
     */
    public BitInputStream(InputStream fis){
        this.fis=fis;
        this.bufferindex=8;
        this.buffer=new boolean[8];
        this.bitsleft=true;
    }

    /**
     * Read data into byte array
     * @param b byte array
     * @return number of bytes read
     * @throws IOException
     */
    public int read(byte[] b) throws IOException{
        for (int x=0;x<b.length;x++){
            boolean[] b2=new boolean[8];
            read(b2);
            b[x]=Basis.booleansToByte(b2);
        }
        return b.length;
    }

    /**
     * Reads a single byte
     * @return byte
     * @throws IOException
     */
    public byte readByte() throws IOException{
        boolean[] b=new boolean[8];
        read(b);
        return Basis.booleansToByte(b);
    }

    /**
     * Read data into boolean array
     * @param b boolean array
     * @return number of bits read
     * @throws IOException
     */
    public int read(boolean[] b) throws IOException{
        //System.out.println(bitsleft+" "+fis.available());
        if (!bitsleft)return 0;
        for (int x=0;x<b.length;x++){
            b[x]=readBit();
            if (!bitsleft)return x;
        }
        return b.length;
    }

    /**
     * Read data into boolean array starting from <offset>, reading <count> bits
     * @param b boolean array
     * @param offset position to start writing into boolean array
     * @param count number of bits to read
     * @return number of bits read
     * @throws IOException
     */
    public int read(boolean[] b, int offset, int count) throws IOException{
        for (int x=offset;x<b.length&&x<offset+count;x++){
            b[x]=readBit();
            if (!bitsleft)return x-offset;
        }
        return b.length;
    }

    /**
     * Read <count> bits into boolean array
     * @param b boolean array
     * @param count number of bits to read
     * @return number of read bits
     * @throws IOException
     */
    public int read(boolean[] b, int count) throws IOException{
        for (int x=0;x<b.length&&x<count;x++){
            b[x]=readBit();
            if (!bitsleft)return x;
        }
        return b.length;
    }

    /**
     * Reads a single bit and returns it
     * @return bit
     * @throws IOException
     */
    public boolean readBit() throws IOException {
        if (bufferindex>7){
            byte[] b=new byte[1];
            int read=fis.read(b,0,1);
            if (read==0)bitsleft=false;
            buffer=Basis.byteToBooleans(b[0]);
            bufferindex=0;
        }
        return buffer[bufferindex++];

    }

    public void close() throws IOException{
        fis.close();
    }

    /**
     * Return number of available bytes for reading
     * @return int
     * @throws IOException
     */
    public int available() throws IOException{
        return fis.available();
    }
}
