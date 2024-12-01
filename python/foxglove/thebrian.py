
from mcap_protobuf.writer import Writer

from foxglove_schemas_protobuf.CameraCalibration_pb2 import CameraCalibration
from foxglove_schemas_protobuf.RawImage_pb2 import RawImage
from foxglove_schemas_protobuf.CompressedImage_pb2 import CompressedImage
from google.protobuf.timestamp_pb2 import Timestamp
# from google.protobuf import timestamp_pb2

from mcap_protobuf.decoder import DecoderFactory
from mcap.reader import make_reader
from collections import deque
from collections import defaultdict

# from msgs.python.msgs_pb2 import RawFullImu, RawImu, Vector, Quaternion, Image
# from msgs.python.msgs_pb2 import RawFullImu, Vector
# from collections import deque


def make_timestamp(time_ns: int) -> Timestamp:
    return Timestamp(seconds=time_ns // 1_000_000_000, nanos=time_ns % 1_000_000_000)

def make_int(timestamp: Timestamp) -> int:
    return timestamp.seconds*1_000_000_000 + timestamp.nanos


def read_mcap(filename):
    """
    Read mcap file

    Args:
        filename
    Return:
        defaultdict(deque)
        dict: metadata of topics
    """
    data = defaultdict(deque)
    meta = {}
    with open(filename, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, proto_msg in reader.iter_decoded_messages():
            # print("-"*60)
            # print(f"{channel.topic} {schema.name} [{message.log_time}]: {proto_msg}")
            # print(f"{channel} {schema} {message}")
            data[channel.topic].append(proto_msg)
            meta[channel.topic] = {"schema": schema.name, "count": len(data[channel.topic])}

    # print(meta)
    return data, meta



def write_mcap(filename, data):
    """
    Write an mcap file

    Args:
        filename
        data: defaultdict(deque)
    """
    with open(filename, "wb") as f, Writer(f) as mcap_writer:
        for topic, deq in data.items():
            for msg in deq:
                ts = make_int(msg.timestamp)
                print(ts)
                mcap_writer.write_message(
                    topic=topic,
                    message=msg,
                    log_time=ts,
                    publish_time=ts,
                )