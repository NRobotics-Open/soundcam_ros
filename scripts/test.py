#! /usr/bin/env python3

import queue
import threading
import time
import selectors

# Function to produce data for a queue
def produce_data(data_queue, stop_event, queue_id):
    while not stop_event.is_set():
        data = (queue_id, time.time())  # Simulate data production with a timestamp
        data_queue.put(data)
        print(f"Produced data on queue {queue_id}: {data}")
        time.sleep(1)
    print(f"Producer {queue_id} terminating...")

# Function to manage multiple queues in a single thread
def manage_queues(queues, stop_event):
    while not stop_event.is_set():
        for q_id, q in queues.items():
            try:
                data = q.get(timeout=0.1)  # Non-blocking get with timeout
                print(f"Managed data from queue {q_id}: {data}")
            except queue.Empty:
                continue
    print("Queue manager terminating...")

if __name__ == "__main__":
    num_queues = 12
    queues = {i: queue.Queue() for i in range(num_queues)}
    stop_event = threading.Event()

    # Start producer threads
    producers = []
    for q_id in queues:
        t = threading.Thread(target=produce_data, args=(queues[q_id], stop_event, q_id))
        t.start()
        producers.append(t)

    # Start a single thread to manage all queues
    queue_manager_thread = threading.Thread(target=manage_queues, args=(queues, stop_event))
    queue_manager_thread.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Main thread received interrupt, setting stop event.")
        stop_event.set()
        for t in producers:
            t.join()
        queue_manager_thread.join()
        print("Main thread terminating...")
