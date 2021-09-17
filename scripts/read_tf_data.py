from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import deserialize_cdr, serialize_cdr
from pathlib import Path
import sys



def check_for_impossible_values(transform):
    max_value = 1e100
    t_dict = {"x": transform.transform.translation.x,
         "y": transform.transform.translation.y,
         "z": transform.transform.translation.z,
         "qw": transform.transform.rotation.w,
         "qx": transform.transform.rotation.x,
         "qy": transform.transform.rotation.y,
         "qz": transform.transform.rotation.z}
    for key, value in t_dict.items():
        if value > max_value:
            print(f"The transform from {transform.header.frame_id} to {transform.child_frame_id} has a too high value in"
                  f" {key}. It is {value}")


def correct_impossible_values(transform):
    max_value = 1e100
    fixed = False
    if transform.transform.translation.x > max_value:
        transform.transform.translation.x = .0
        fixed = True
    if transform.transform.translation.y > max_value:
        transform.transform.translation.y = .0
        fixed = True
    if transform.transform.translation.z > max_value:
        transform.transform.translation.z = .0
        fixed = True
    if fixed:
        return 1
    else:
        return 0


def print_transform(transform):
    t_dict = {"x": transform.transform.translation.x,
         "y": transform.transform.translation.y,
         "z": transform.transform.translation.z,
         "qw": transform.transform.rotation.w,
         "qx": transform.transform.rotation.x,
         "qy": transform.transform.rotation.y,
         "qz": transform.transform.rotation.z}
    print(t_dict)

def rewrite_rosbag(bag_path, new_bag_path):
    # create reader instance and open for reading
    with Reader(bag_path) as reader:
        with Writer(new_bag_path) as writer:
            # iterate over messages

            errors = 0
            for connection, timestamp, rawdata in reader.messages():
                msg = deserialize_cdr(rawdata, connection.msgtype)
                if connection.topic == '/tf' or connection.topic == '/tf_static':
                    for transform in msg.transforms:
                        if correct_impossible_values(transform):
                            errors += 1
                found = False
                if len(writer.connections) == 0:
                    new_connection = writer.add_connection(connection.topic, connection.msgtype,
                                                               connection.serialization_format,
                                                               connection.offered_qos_profiles)
                else:
                    for key, value in writer.connections.items():
                        if value.topic == connection.topic:
                            new_connection = value
                            found = True
                            break
                    if not found:
                        new_connection = writer.add_connection(connection.topic, connection.msgtype,
                                                                   connection.serialization_format,
                                                                   connection.offered_qos_profiles)
                # if not (connection in writer.connections.values()):
                #     new_connection = writer.add_connection(connection.topic, connection.msgtype,
                #                                            connection.serialization_format,
                #                                            connection.offered_qos_profiles)
                # else:
                #     new_connection = connection
                # new_connection = writer.add_connection(connection.topic, connection.msgtype,
                #                                        connection.serialization_format,
                #                                        connection.offered_qos_profiles)
                # serialize and write message
                writer.write(new_connection, timestamp, serialize_cdr(msg, new_connection.msgtype))
    print(f"{errors} errors fixed")





def main(bag_path):
    # create reader instance and open for reading
    with Reader(bag_path) as reader:
        # topic and msgtype information is available on .connections dict
        # for connection in reader.connections.values():
        #     print(connection.topic, connection.msgtype)

        # iterate over messages
        tf = []
        tf_static = []
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/tf' or connection.topic == '/tf_static':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                for transform in msg.transforms:
                    child = transform.child_frame_id
                    frame = transform.header.frame_id
                    check_for_impossible_values(transform)
                    if connection.topic == '/tf':
                        tf.append((frame, child))
                    else:
                        tf_static.append((frame, child))
        s = set(tf)
        print(f"There are {len(s)} unique tf:\n{s}")
        s = set(tf_static)
        print(f"There are {len(s)} unique static tf:\n{s}")

        # messages() accepts connection filters
        # connections = [x for x in reader.connections.values() if x.topic == '/imu_raw/Imu']
        # for connection, timestamp, rawdata in reader.messages(connections=connections):
        #     msg = deserialize_cdr(rawdata, connection.msgtype)
        #     print(msg.header.frame_id)

if __name__ == "__main__":
    path = sys.argv[1]
    bag_path = Path(path)
    new_bag_path = bag_path.parent / (bag_path.name + "_fixed")
    rewrite_rosbag(bag_path, new_bag_path)
    #main(new_bag_path)
