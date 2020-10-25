package messages.kafka;

import java.util.LinkedList;

public class WorkerStatus {
    public WorkerStatus() {
    }

    public WorkerStatus(String workerName, LinkedList<String> processedTask) {
        this.processedTask = processedTask;
        this.workerName = workerName;
    }
    private String workerName = new String();
    private LinkedList<String> processedTask = new LinkedList<String>();

    public String getWorkerName()
    {
        return this.workerName;
    }

    public LinkedList<String> getProcessedTask()
    {
        return this.processedTask;
    }
}

