import org.apache.kafka.clients.consumer.*;
import org.apache.kafka.common.serialization.LongDeserializer;

import java.time.Duration;
import java.util.Collections;
import java.util.Properties;

import messages.kafka.*;

/**
 * Apache Kafka - Building Kafka Consumers, Scalability and Pub/Sub
 */
public class Application {
    private static final String TOPIC = "workers_status";
    private static final String BOOTSTRAP_SERVERS = "localhost:9092,localhost:9093,localhost:9094";

    public static void main(String[] args) {
        String consumerGroup = "defaultConsumerGroup";
        if (args.length == 1) {
            consumerGroup = args[0];
        }

        System.out.println("Consumer is part of consumer group " + consumerGroup);
        Consumer<Long, WorkerStatus> kafkaConsumer = createKafkaConsumer(BOOTSTRAP_SERVERS, consumerGroup);
        consumeMessages(TOPIC, kafkaConsumer);
    }

    public static void consumeMessages(String topic, Consumer<Long, WorkerStatus> kafkaConsumer) {
        kafkaConsumer.subscribe(Collections.singletonList(topic));

        while (true) {
            ConsumerRecords<Long, WorkerStatus> consumerRecords = kafkaConsumer.poll(Duration.ofSeconds(1));
            if (consumerRecords.isEmpty()) {
                // do something else
            }

            for (ConsumerRecord<Long, WorkerStatus> record : consumerRecords) {
                if(false) {
                    System.out.println(String.format("Received record (key: %d, value: %s, partition: %d, offset: %d",
                        record.key(), record.value().toString(), record.partition(), record.offset()));
                }
                System.out.println(String.format("Status for task %s", record.value().getWorkerName()));
                System.out.println("current processed tasks:");
                String tasksInfo = String.join(",", record.value().getProcessedTask());
                System.out.println(tasksInfo);
            }

            // do something with the records

            kafkaConsumer.commitAsync();
        }
    }

    public static Consumer<Long, WorkerStatus> createKafkaConsumer(String bootstrapServers, String consumerGroup) {
        Properties properties = new Properties();

        properties.put(ConsumerConfig.BOOTSTRAP_SERVERS_CONFIG, bootstrapServers);
        properties.put(ConsumerConfig.KEY_DESERIALIZER_CLASS_CONFIG, LongDeserializer.class.getName());
        properties.put(ConsumerConfig.VALUE_DESERIALIZER_CLASS_CONFIG, WorkerStatusDeserializer.class.getName());
        properties.put(ConsumerConfig.GROUP_ID_CONFIG, consumerGroup);
        properties.put(ConsumerConfig.ENABLE_AUTO_COMMIT_CONFIG, false);

        return new KafkaConsumer<>(properties);
    }

}
