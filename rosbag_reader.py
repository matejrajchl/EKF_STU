import rosbag

class RosBagReader:
    def __init__(self, bag_file):
        self.bag_file = bag_file
        self.callbacks = {}
        
    def read_all_names(self):
        bag = rosbag.Bag(self.bag_file)
        topic_names = set()

        for topic, _, _ in bag.read_messages():
            topic_names.add(topic)

        bag.close()

        print("List of topic names:")
        for topic_name in topic_names:
            print(topic_name)

    def register_callback(self, topic, callback):
        self.callbacks[topic] = callback

    def start_reading(self):
        bag = rosbag.Bag(self.bag_file)
        i = 0
        for topic, msg, t in bag.read_messages():
            if topic in self.callbacks:
                self.callbacks[topic](msg, t)
            #if i > 1000:
            #    return
            #else:
            #    i = i +1

        bag.close()

