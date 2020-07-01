package messages.kafka;


import java.util.Map;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.apache.kafka.common.serialization.Deserializer;
import org.apache.kafka.common.serialization.Serializer;

public class WorkerStatusSerializer implements Serializer<WorkerStatus> {
    private ObjectMapper objectMapper = new ObjectMapper();

    @Override
    public void configure(Map<String, ?> configs, boolean isKey)
    {
    }

    @Override
    public byte[] serialize(String topic, WorkerStatus message)
    {
        if (message == null) {
            return null;
        }

        try {
            return objectMapper.writeValueAsBytes(message);
        } catch (JsonProcessingException e) {
            e.printStackTrace();
            return null;
        }
    }

    @Override
    public void close()
    {
    }
}
