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

import org.apache.kafka.clients.consumer.*;
import org.apache.kafka.common.serialization.LongDeserializer;
import org.apache.kafka.common.serialization.StringDeserializer;

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
                System.out.println(String.format("Received record (key: %d, value: %s, partition: %d, offset: %d",
                        record.key(), record.value().toString(), record.partition(), record.offset()));

                System.out.println(String.format("Status for task %s", record.value().getWorkerName()));
                System.out.println("current tasks:");
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
