package messages.kafka;

import java.util.LinkedList;

public class WorkerStatus {
    public WorkerStatus() {
        this.processedTask = new LinkedList<String>();
        this.workerName = "test";
    }

    public WorkerStatus(LinkedList<String> processedTask) {
        this.processedTask = new LinkedList<String>();
        this.workerName = "test";
    }
    private String workerName;
    private LinkedList<String> processedTask;

    public String getWorkerName()
    {
        return this.workerName;
    }

    public LinkedList<String> getProcessedTask()
    {
        return this.processedTask;
    }
}

