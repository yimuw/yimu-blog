package messages.kafka;

import java.util.LinkedList;

public class WorkerStatus {

    public WorkerStatus(String workerName, LinkedList<String> processedTask) {
        this.processedTask = processedTask;
        this.workerName = workerName;
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

