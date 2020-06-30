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

import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpContext;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;

import java.io.*;
import java.net.InetSocketAddress;

import java.util.concurrent.Executors;
import messages.ComputingTask;

public class Worker {
    private static final String TASK_ENDPOINT = "/task";
    private static final String STATUS_ENDPOINT = "/status";

    private final int port;
    private HttpServer server;

    public void startServer(int serverPort)
    {
        Worker webServer = new Worker(serverPort);
        webServer.startServer();

        System.out.println("Server is listening on port " + serverPort);
    }

    public Worker(int port)
    {
        this.port = port;
    }

    public void startServer()
    {
        try {
            this.server = HttpServer.create(new InetSocketAddress(port), 0);
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }

        HttpContext statusContext = server.createContext(STATUS_ENDPOINT);
        HttpContext taskContext = server.createContext(TASK_ENDPOINT);

        statusContext.setHandler(this ::handleStatusCheckRequest);
        taskContext.setHandler(this ::handleTaskRequest);

        server.setExecutor(Executors.newFixedThreadPool(1));
        server.start();

        System.out.println("server start!");
    }

    boolean checkHeaderHas(Headers headers, String headerName)
    {
        if (headers.containsKey(headerName)) {
            System.out.println(headerName + ":" + headers.get(headerName).get(0));
            return true;
        } else {
            System.out.println("request doesn't contain " + headerName);
            return false;
        }
    }

    class TaskStatus {
        TaskStatus(byte[] result, boolean success)
        {
            this.result = result;
            this.success = success;
        }

        byte[] result;
        boolean success;
    }

    private void handleTaskRequest(HttpExchange exchange) throws IOException
    {
        if (!exchange.getRequestMethod().equalsIgnoreCase("post")) {
            exchange.close();
            return;
        }

        System.out.println("============ Handling new computing request =============");

        byte[] requestBytes = exchange.getRequestBody().readAllBytes();

        messages.ComputingTask task = messages.ComputingTask.deserialize(requestBytes);

        TaskStatus status = calculateResponse(task.bin, task.taskId, task.binName, task.args);
        if (status.success) {
            sendResponse(status.result, exchange, 200);
        } else {
            sendResponse(status.result, exchange, 400);
        }

        System.out.println("============ Done handling request =============");
    }

    public static boolean deleteDirectory(File dir)
    {
        if (dir.isDirectory()) {
            File[] children = dir.listFiles();
            for (int i = 0; i < children.length; i++) {
                boolean success = deleteDirectory(children[i]);
                if (!success) {
                    return false;
                }
            }
        }
        System.out.println("removing file or directory : " + dir.getName());
        return dir.delete();
    }

    private void setUp(byte[] requestBytes, String taskDir, String binName) throws IOException
    {
        File resdir = new File(taskDir);
        if (resdir.exists()) {
            System.out.println("delete exist result dir");
            deleteDirectory(resdir);
        }
        resdir.mkdir();

        String binSavePath = taskDir + "/" + binName;
        System.out.println("saving bin at:" + binSavePath);

        OutputStream outputStream = new FileOutputStream(binSavePath);
        outputStream.write(requestBytes);
        outputStream.close();
        System.out.println("bin received and saved");
    }

    private byte[] postProcess(String taskDir) throws IOException
    {
        String RESULT_FILE = "output.txt";
        String resultFilePath = taskDir + "/" + RESULT_FILE;
        InputStream inputStream = new FileInputStream(resultFilePath);
        long fileSize = new File(resultFilePath).length();
        byte[] allBytes = new byte[(int)fileSize];
        inputStream.read(allBytes);
        inputStream.close();
        return allBytes;
    }

    private TaskStatus calculateResponse(byte[] requestBytes, String taskDir, String binName, String binArgs)
    {
        try {
            setUp(requestBytes, taskDir, binName);
            startCommand(taskDir, binName, binArgs);
            return new TaskStatus(postProcess(taskDir), true);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            return new TaskStatus(new byte[0], false);
        }
    }

    private void startCommand(String taskDir, String binName, String binArgs) throws IOException, InterruptedException
    {
        File file = new File(binName);

        String command = "java -jar " + file.getName() + " " + binArgs;
        String fullCommand = String.format("cd %s; %s", taskDir, command);
        System.out.println(String.format("Launching worker instance : %s ", fullCommand));
        String[] cmd1 = { "/bin/sh", "-c", fullCommand };
        Process p1 = Runtime.getRuntime().exec(cmd1);
        p1.waitFor();

        System.out.println("process finish!");
    }

    private void handleStatusCheckRequest(HttpExchange exchange) throws IOException
    {
        if (!exchange.getRequestMethod().equalsIgnoreCase("get")) {
            exchange.close();
            return;
        }

        String responseMessage = "Server is alive\n";
        sendResponse(responseMessage.getBytes(), exchange, 200);
    }

    private void sendResponse(byte[] responseBytes, HttpExchange exchange, int code) throws IOException
    {
        exchange.sendResponseHeaders(code, responseBytes.length);
        OutputStream outputStream = exchange.getResponseBody();
        outputStream.write(responseBytes);
        outputStream.flush();
        outputStream.close();
        exchange.close();
    }
}
