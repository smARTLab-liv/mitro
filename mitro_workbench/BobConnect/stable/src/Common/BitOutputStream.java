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
public class BitOutputStream {
    private OutputStream fos;
    private boolean[] buffer;
    private int bufferindex;


    /**
     * Construct stream from file
     * @param f file
     * @throws java.io.FileNotFoundException
     */
    public BitOutputStream(File f) throws FileNotFoundException {
        this.fos=new FileOutputStream(f);
        this.bufferindex=0;
        this.buffer=new boolean[8];
    }

    /**
     * Construct stream from inputstream
     * @param fos fileinputstream
     */
    public BitOutputStream(OutputStream fos){
        this.fos=fos;
        this.bufferindex=0;
        this.buffer=new boolean[8];
    }

    /**
     * Write bytes to file
     * @param b byte array
     * @throws IOException
     */
    public void write(byte[] b) throws IOException{
        for (int x=0;x<b.length;x++)
            write(Basis.byteToBooleans(b[x]));
    }

    /**
     * Write byte to file
     * @param b byte array
     * @throws IOException
     */
    public void write(byte b) throws IOException{
        write(Basis.byteToBooleans(b));
    }


    /**
     * Write bits to file
     * @param b boolean array
     * @throws IOException
     */
    public void write(boolean[] b) throws IOException{
        for (int x=0;x<b.length;x++)addBit(b[x]);
    }

    /**
     * Flush remaining data in buffer to file (may write some empty bits)
     * @throws IOException
     */
    public void flush() throws IOException{
        if (bufferindex>0){
            byte out=Basis.booleansToByte(buffer);
            bufferindex=0;
            fos.write(out);
        }
    }

    public void close() throws IOException{
        flush();
        fos.close();
    }

    /**
     * Writes a single but to file
     * @param b bit
     * @throws IOException
     */
    private void addBit(boolean b) throws IOException {
        buffer[bufferindex]=b;
        bufferindex++;
        if (bufferindex>7){
            byte out=Basis.booleansToByte(buffer);
            bufferindex=0;
            fos.write(out);
            //fos.flush();
        }
    }
}
