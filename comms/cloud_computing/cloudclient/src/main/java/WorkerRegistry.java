import org.apache.zookeeper.*;
import org.apache.zookeeper.data.Stat;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class WorkerRegistry  implements Watcher  {
    private static final String ZOOKEEPER_ADDRESS = "localhost:2181";
    private static final int SESSION_TIMEOUT = 3000;
    private static final String REGISTRY_ZNODE = "/workers";
    private ZooKeeper zooKeeper;

    public WorkerRegistry() throws IOException
    {
        zooKeeper = new ZooKeeper(ZOOKEEPER_ADDRESS, SESSION_TIMEOUT, this);
    }

    public synchronized List<String> getAllServiceAddresses() throws KeeperException, InterruptedException, IOException {
        List<String> workerZnodes = zooKeeper.getChildren(REGISTRY_ZNODE, false);

        List<String> addresses = new ArrayList<>(workerZnodes.size());

        for (String workerZnode : workerZnodes) {
            String workerFullPath = REGISTRY_ZNODE + "/" + workerZnode;
            Stat stat = zooKeeper.exists(workerFullPath, false);
            if (stat == null) {
                continue;
            }

            byte[] addressBytes = zooKeeper.getData(workerFullPath, false, stat);
            String address = new String(addressBytes);
            addresses.add(address);
        }

        List<String> allServiceAddresses = Collections.unmodifiableList(addresses);

        
        return allServiceAddresses;
    }

    @Override
    public void process(WatchedEvent event) {
        switch (event.getType()) {
            case None:
                if (event.getState() == Event.KeeperState.SyncConnected) {
                    System.out.println("Successfully connected to Zookeeper");
                } else {
                    synchronized (zooKeeper) {
                        System.out.println("Disconnected from Zookeeper event");
                        zooKeeper.notifyAll();
                    }
                }
        }
    }
}
