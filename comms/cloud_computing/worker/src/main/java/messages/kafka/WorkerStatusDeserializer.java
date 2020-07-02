package messages.kafka;

import java.util.Map;

import com.fasterxml.jackson.databind.ObjectMapper;

import org.apache.kafka.common.serialization.Deserializer;

public class WorkerStatusDeserializer implements Deserializer<WorkerStatus> {

    private ObjectMapper objectMapper = new ObjectMapper();

    @Override
    public void configure(Map<String, ?> configs, boolean isKey)
    {
    }

    @Override
    public WorkerStatus deserialize(String topic, byte[] data)
    {
        try {
            return objectMapper.readValue(new String(data, "UTF-8"), WorkerStatus.class);
        } catch (Exception e) {
            System.out.println("Unable to deserialize message");
            return null;
        }
    }

    @Override
    public void close()
    {
    }
}