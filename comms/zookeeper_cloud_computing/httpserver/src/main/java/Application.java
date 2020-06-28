import org.apache.zookeeper.KeeperException;
import org.apache.zookeeper.WatchedEvent;
import org.apache.zookeeper.Watcher;
import org.apache.zookeeper.ZooKeeper;

import java.io.IOException;
import java.net.InetAddress;

public class Application implements Watcher {
    private static final String ZOOKEEPER_ADDRESS = "localhost:2181";
    private static final int SESSION_TIMEOUT = 3000;
    private static final int DEFAULT__SERVER_PORT = 8081;
    private ZooKeeper zooKeeper;

    public static void main(String[] args) throws IOException, InterruptedException, KeeperException
    {
        int serverPort = args.length == 1 ? Integer.parseInt(args[0]) : DEFAULT__SERVER_PORT;
        Application application = new Application();
        ZooKeeper zooKeeper = application.connectToZookeeper();
        ServiceRegistry serviceRegistry = new ServiceRegistry(zooKeeper);

        WebServer server = new WebServer(serverPort);
        server.startServer();
        String currentServerAddress = String.format("http://%s:%d", InetAddress.getLocalHost().getCanonicalHostName(), serverPort);

        // use zookeeper to track address of servers
        serviceRegistry.registerToCluster(currentServerAddress);

        application.run();
        application.close();
        System.out.println("Disconnected from Zookeeper, exiting application");
    }

    public ZooKeeper connectToZookeeper() throws IOException
    {
        this.zooKeeper = new ZooKeeper(ZOOKEEPER_ADDRESS, SESSION_TIMEOUT, this);
        return zooKeeper;
    }

    public void run() throws InterruptedException
    {
        synchronized (zooKeeper)
        {
            zooKeeper.wait();
        }
    }

    public void close() throws InterruptedException
    {
        zooKeeper.close();
    }

    @Override
    public void process(WatchedEvent event)
    {
        switch (event.getType()) {
        case None:
            if (event.getState() == Event.KeeperState.SyncConnected) {
                System.out.println("Successfully connected to Zookeeper");
            } else {
                synchronized (zooKeeper)
                {
                    System.out.println("Disconnected from Zookeeper event");
                    zooKeeper.notifyAll();
                }
            }
        }
    }
}
