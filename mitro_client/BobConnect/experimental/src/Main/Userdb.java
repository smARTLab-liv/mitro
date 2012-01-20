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

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.math.BigInteger;
import java.security.MessageDigest;

public class Userdb {
    private File db;

    public Userdb(File db) {
        this.db = db;
    }

    public boolean checkCredentials(String username, String password, String salt) throws IOException {
        BufferedReader reader=new BufferedReader(new FileReader(db));
        String line=reader.readLine();
        while (line!=null){
            line=line.trim();
            if (!line.startsWith("#")){
                int i=line.indexOf(":");
                if (i>0){
                    if (line.substring(0,i).equals(username)){
                        try{
                            MessageDigest md=MessageDigest.getInstance("SHA-256");

                            String sha256=new BigInteger(1,md.digest((line.substring(i+1)+salt).getBytes("UTF8"))).toString(16);

                            if (sha256.trim().equals(password))return true;
                        }
                        catch(Exception ex){}

                    }
                }
            }
            line=reader.readLine();
        }
        return false;
    }

}
