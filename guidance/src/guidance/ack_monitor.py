import argparse
import json

from guidance.bridge import MISSION_ACK_TOPIC
from guidance.wire_format import WIRE_FORMAT_JSON, ack_to_mapping, decode_ack


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor guidance mission acknowledgements")
    parser.add_argument(
        "--topic",
        default=MISSION_ACK_TOPIC,
        help="Zenoh topic for mission acknowledgement payloads",
    )
    parser.add_argument(
        "--wire-format",
        default=WIRE_FORMAT_JSON,
        choices=["json", "flexbuffers"],
        help="Payload wire format for ack decoding",
    )
    return parser.parse_args()


def main():
    import zenoh

    args = _parse_args()
    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    subscriber = session.declare_subscriber(args.topic)
    print(f"Monitoring acknowledgements on {args.topic} ({args.wire_format})")

    try:
        while True:
            sample = subscriber.recv()
            payload = sample.payload
            if args.wire_format == WIRE_FORMAT_JSON:
                raw_payload = payload.to_string()
            else:
                to_bytes = getattr(payload, "to_bytes", None)
                raw_payload = to_bytes() if callable(to_bytes) else payload.to_string().encode("utf-8")

            ack = decode_ack(raw_payload, args.wire_format)
            print(json.dumps(ack_to_mapping(ack), sort_keys=True))
    finally:
        session.close()


if __name__ == "__main__":
    main()
