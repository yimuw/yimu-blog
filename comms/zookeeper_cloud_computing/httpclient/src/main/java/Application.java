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

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.apache.zookeeper.KeeperException;

public class Application {
    private static final String WORKER_ADDRESS_1 = "http://localhost:8081/task";
    private static final String WORKER_ADDRESS_2 = "http://localhost:8082/task";

    static private byte[] readData(String fileName)
    {
        try {
            InputStream inputStream = new FileInputStream(fileName);
            long fileSize = new File(fileName).length();
            byte[] allBytes = new byte[(int)fileSize];

            inputStream.read(allBytes);

            System.out.println("file size: " + fileSize);

            return allBytes;

        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        return new byte[0];
    }

    static private List<String> getWorkAddress()
    {
        try {
            WorkerRegistry workerRegistry = new WorkerRegistry();
            return workerRegistry.getAllServiceAddresses();
        } catch (KeeperException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        return Collections.emptyList();
    }

    public static void main(String[] args){
        Application app = new Application();

        
        List<String> workAddresses = getWorkAddress();
        for(String s: workAddresses) {
            System.out.println("worker address: " + s);
        }            

        String binName = args[0];
        String argName = args[1];
        byte[] requestPayload = readData(binName);
        
        WebClient webClient = new WebClient();
        CompletableFuture<String> future = webClient.sendTask(WORKER_ADDRESS_1, binName, argName, requestPayload);
        
        try {
            future.join();
            String result = future.get();
            System.out.println("recv : " + result);
            System.out.println("file size: " + result.getBytes(StandardCharsets.US_ASCII).length);
            
            OutputStream outputStream = new FileOutputStream("recv_result.zip");
            byte[] bytes = result.getBytes(StandardCharsets.US_ASCII);
            outputStream.write(bytes);
            outputStream.close();              
        } catch (Exception e) {
            //TODO: handle exception
            e.printStackTrace();
            System.out.println("exception!");
        }
    }
}
