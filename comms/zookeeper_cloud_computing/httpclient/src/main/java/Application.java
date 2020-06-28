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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.io.*;
import java.math.BigInteger;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

import org.apache.zookeeper.KeeperException;

class Task {
    Task(String id, String bin, String arg)
    {
        this.id = id;
        this.binName = bin;
        this.argName = arg;
    }
    String id;
    String binName;
    String argName;

    CompletableFuture<String> serverResponse;
    String resultPath;
}

public class Application {
    static private byte[] readData(String fileName) throws IOException
    {
        InputStream inputStream = new FileInputStream(fileName);
        long fileSize = new File(fileName).length();
        byte[] allBytes = new byte[(int)fileSize];

        inputStream.read(allBytes);
        inputStream.close();
        return allBytes;
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

    public static void main(String[] args) throws Exception
    {
        List<String> workAddresses = getWorkAddress();
        if (workAddresses.size() == 0) {
            System.out.println("no server available!");
            return;
        }
        for (String s : workAddresses) {
            System.out.println("worker address: " + s);
        }

        ArrayList<Task> tasks = new ArrayList<Task>();
        final String pathToBin = "./addIntegers-1.0-SNAPSHOT-jar-with-dependencies.jar"; 
        tasks.add(new Task("task1", pathToBin, "1 1000"));
        tasks.add(new Task("task2", pathToBin, "500 10000"));
        tasks.add(new Task("task3", pathToBin, "5001 1000000"));
        tasks.add(new Task("task4", pathToBin, "1 10000000"));
        tasks.add(new Task("task5", pathToBin, "500 1000"));

        for (int i = 0; i < tasks.size(); ++i) {
            Task task = tasks.get(i);
            byte[] requestPayload = readData(task.binName);
            WebClient webClient = new WebClient();
            System.out.println(String.format("send task [%s] to server", task.id));
            task.serverResponse = webClient.sendTask(workAddresses.get(i % workAddresses.size()) + "/task",
                task.id, task.binName, task.argName, requestPayload);
        }

        new File("results").mkdirs();

        for (int i = 0; i < tasks.size(); ++i) {
            Task task = tasks.get(i);
            try {
                task.serverResponse.join();
                String result = task.serverResponse.get();
                System.out.println(String.format("recv result for task : %s", task.id));
                String taskDir = String.format("results/task%d", i);

                new File(taskDir).mkdirs();
                String resultFilePath = taskDir + "/result.txt";
                OutputStream outputStream = new FileOutputStream(resultFilePath);
                byte[] bytes = result.getBytes(StandardCharsets.US_ASCII);
                outputStream.write(bytes);
                outputStream.close();
                task.resultPath = resultFilePath;
            } catch (IOException | InterruptedException | ExecutionException e) {
                System.out.println("exception when connect to servers");
                e.printStackTrace();
                throw e;
            }
        }

        BigInteger sum = new BigInteger("0");
        for (int i = 0; i < tasks.size(); ++i) {
            Task task = tasks.get(i);
            File file = new File(task.resultPath);
            FileReader fr;
            try {
                fr = new FileReader(file);
                BufferedReader br = new BufferedReader(fr);
                String line;
                line = br.readLine();
                BigInteger subResult = new BigInteger(line);
                sum = sum.add(subResult);
            } catch (IOException e) {
                e.printStackTrace();
                throw e;
            }
        }

        System.out.println("sum: " + sum);
    }
}
