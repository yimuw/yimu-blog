package messages;

import java.io.IOException;
import java.io.UnsupportedEncodingException;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class ComputingTask {
    public byte[] bin;
    public String args;
    public String binName;
    public String taskId;

    public ComputingTask()
    {
    }
    public ComputingTask(byte[] bin, String args, String binName, String taskId)
    {
        this.bin = bin;
        this.args = args;
        this.binName = binName;
        this.taskId = taskId;
    }

    static public ComputingTask deserialize(byte[] data) throws JsonParseException, JsonMappingException, UnsupportedEncodingException, IOException
    {
        ObjectMapper objectMapper = new ObjectMapper();
        return objectMapper.readValue(new String(data, "UTF-8"), ComputingTask.class);
    }

    static public byte[] serialize(ComputingTask message) throws JsonProcessingException
    {
        if (message == null) {
            return null;
        }

        ObjectMapper objectMapper = new ObjectMapper();
        return objectMapper.writeValueAsBytes(message);
    }
}
