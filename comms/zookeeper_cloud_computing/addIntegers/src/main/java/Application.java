/*
 *  MIT License
 *
 *  Copyright (c) 2019 Michael Pogrebinsky - Distributed Systems & Cloud Computing with Java
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

import java.io.*;
import java.math.BigInteger;

public class Application {
    public static void main(String[] args) {
        try {
            BigInteger start = new BigInteger(args[0]);
            BigInteger end = new BigInteger(args[1]);
            
            BigInteger sum = new BigInteger("0");
            for(; start.compareTo(end) < 0; start = start.add(BigInteger.ONE)) {
                sum = sum.add(start);
            }
            System.out.println("sum: " + sum);

            PrintWriter p = new PrintWriter(new FileOutputStream("output.txt", false));
            p.println(sum.toString());
            p.close();
            System.out.println("file saved");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
