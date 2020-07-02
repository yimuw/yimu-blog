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
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

import org.apache.zookeeper.KeeperException;
import messages.ComputingTask;

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
        final String PATH_TO_BIN = "./addIntegers-1.0-SNAPSHOT-jar-with-dependencies.jar";
        tasks.add(new Task("task1", PATH_TO_BIN, "1 100000000"));
        tasks.add(new Task("task2", PATH_TO_BIN, "500 100000000"));
        tasks.add(new Task("task3", PATH_TO_BIN, "5001 10000000"));
        tasks.add(new Task("task4", PATH_TO_BIN, "1 20000000"));
        tasks.add(new Task("task5", PATH_TO_BIN, "500 20000000"));

        for (int i = 0; i < tasks.size(); ++i) {
            Task task = tasks.get(i);

            task.id += "-" + System.currentTimeMillis();

            byte[] bin = readData(task.binName);
            ComputingTask computingTask = new ComputingTask(bin, task.argName, task.binName, task.id);
            WebClient webClient = new WebClient();
            System.out.println(String.format("send task [%s] to server [%s]", task.id, workAddresses.get(i % workAddresses.size())));
            task.serverResponse = webClient.sendTask(workAddresses.get(i % workAddresses.size()) + "/task",
                ComputingTask.serialize(computingTask));
        }

        BigInteger sum = new BigInteger("0");
        for (int i = 0; i < tasks.size(); ++i) {
            Task task = tasks.get(i);
            try {
                task.serverResponse.join();
                String result = task.serverResponse.get();
                System.out.println(String.format("recv result for task : %s", task.id));
                BigInteger subResult = new BigInteger(result.replace("\n", ""));
                sum = sum.add(subResult);
            } catch (InterruptedException | ExecutionException e) {
                System.out.println("exception when getting response from servers");
                e.printStackTrace();
                ;
                throw e;
            }
        }

        System.out.println("sum: " + sum);
    }
}