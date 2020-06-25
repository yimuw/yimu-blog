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

import java.nio.charset.StandardCharsets;
// import java.io.File;
import java.nio.file.*;

import java.io.*;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

public class WebServer {
    private static final String TASK_ENDPOINT = "/task";
    private static final String STATUS_ENDPOINT = "/status";
    private static String resultDir = "result";

    private final int port;
    private HttpServer server;

    public void startServer(int serverPort)
    {
        WebServer webServer = new WebServer(serverPort);
        webServer.startServer();

        System.out.println("Server is listening on port " + serverPort);
    }

    public WebServer(int port)
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

        server.setExecutor(Executors.newFixedThreadPool(8));
        server.start();

        System.out.println("server start!");
    }

    private void handleTaskRequest(HttpExchange exchange) throws IOException
    {
        if (!exchange.getRequestMethod().equalsIgnoreCase("post")) {
            exchange.close();
            return;
        }

        Headers headers = exchange.getRequestHeaders();

        boolean isDebugMode = false;
        if (headers.containsKey("X-Debug") && headers.get("X-Debug").get(0).equalsIgnoreCase("true")) {
            isDebugMode = true;
        }

        if (headers.containsKey("binName")) {
            System.out.println("binName:" + headers.get("binName").get(0));
            System.out.println("binArgs:" + headers.get("binArgs").get(0));
        }

        long startTime = System.nanoTime();

        byte[] requestBytes = exchange.getRequestBody().readAllBytes();

        String binName = headers.get("binName").get(0);
        String binArgs = headers.get("binArgs").get(0);
        byte[] responseBytes = calculateResponse(requestBytes, binName, binArgs);

        long finishTime = System.nanoTime();

        if (isDebugMode) {
            String debugMessage = String.format("Operation took %d ns", finishTime - startTime);
            exchange.getResponseHeaders().put("X-Debug-Info", Arrays.asList(debugMessage));
        }

        sendResponse(responseBytes, exchange);
    }

    private String pathInResultDir(String s)
    {
        return resultDir + "/" + s;
    }

    private static void zipFiles(String zipFilePath, String[] filePaths)
    {
        try {
            File firstFile = new File(zipFilePath);
            String zipFileName = firstFile.getName();

            FileOutputStream fos = new FileOutputStream(zipFileName);
            ZipOutputStream zos = new ZipOutputStream(fos);

            for (String aFile : filePaths) {
                zos.putNextEntry(new ZipEntry(new File(aFile).getName()));

                byte[] bytes = Files.readAllBytes(Paths.get(aFile));
                zos.write(bytes, 0, bytes.length);
                zos.closeEntry();
            }

            zos.close();

        } catch (FileNotFoundException ex) {
            System.err.println("A file does not exist: " + ex);
        } catch (IOException ex) {
            System.err.println("I/O error: " + ex);
        }
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

        // either file or an empty directory
        System.out.println("removing file or directory : " + dir.getName());
        return dir.delete();
    }

    private byte[] calculateResponse(byte[] requestBytes, String binName, String binArgs)
    {
        try {
            File resdir = new File(resultDir);
            if (resdir.exists()) {
                deleteDirectory(resdir);
            }
            resdir.mkdir();

            OutputStream outputStream = new FileOutputStream(pathInResultDir(binName));
            outputStream.write(requestBytes);
            outputStream.close();

            System.out.println("file received and saved");

            File folder = new File(resultDir);
            String[] files = folder.list().clone();

            startCommand(binName, binArgs);

            String[] newFiles = folder.list();

            Set<String> ad = new HashSet<String>(Arrays.asList(files));
            Set<String> s = new HashSet<String>(Arrays.asList(newFiles));
            s.removeAll(ad);

            String[] filesToSend = Arrays.copyOf(s.toArray(), s.size(), String[].class);

            for (String file : filesToSend) {
                System.out.println("file to send: " + file);
            }

            String resultFile = "result.zip";
            zipFiles(resultFile, filesToSend);
            InputStream inputStream = new FileInputStream(resultFile);
            long fileSize = new File(resultFile).length();
            byte[] allBytes = new byte[(int)fileSize];
            System.out.println("file size: " + fileSize);
            inputStream.read(allBytes);
            inputStream.close();
            return allBytes;
        } catch (IOException e) {
            e.printStackTrace();
            return new String("file received and fail to save !\n").getBytes();
        }

        // return new String("file received !\n").getBytes();
    }

    private void startCommand(String binName, String binArgs) throws IOException
    {
        File file = new File(binName);

        try {
            String command = "java -jar " + file.getName() + " " + binArgs;
            String fullCommand = String.format("cd %s; %s", resultDir, command);
            System.out.println(String.format("Launching worker instance : %s ", fullCommand));
            String[] cmd1 = { "/bin/sh", "-c", fullCommand };
            Process p1 = Runtime.getRuntime().exec(cmd1);
            p1.waitFor();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        System.out.println("process finish!");
    }

    private void handleStatusCheckRequest(HttpExchange exchange) throws IOException
    {
        if (!exchange.getRequestMethod().equalsIgnoreCase("get")) {
            exchange.close();
            return;
        }

        String responseMessage = "Server is alive\n";
        sendResponse(responseMessage.getBytes(), exchange);
    }

    private void sendResponse(byte[] responseBytes, HttpExchange exchange) throws IOException
    {
        exchange.sendResponseHeaders(200, responseBytes.length);
        OutputStream outputStream = exchange.getResponseBody();
        outputStream.write(responseBytes);
        outputStream.flush();
        outputStream.close();
        exchange.close();
    }
}
