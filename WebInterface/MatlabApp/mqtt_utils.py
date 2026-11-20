import json

import paho.mqtt.publish as publish

BROKER_HOST = "127.0.0.1"
BROKER_PORT = 1885
TOPIC = "pendulum/cmd"


def send_command(payload, user=None):
    # Send payload as either JSON or plain string to MQTT broker depending on the command

    # If payload is just a string, wrap it
    if isinstance(payload, str):
        payload = {
            "command": payload
        }

    # Attach user info
    if user:
        payload["user_id"] = user.id
        payload["username"] = user.username

    try:
        publish.single(
            TOPIC,
            payload=json.dumps(payload),
            hostname=BROKER_HOST,
            port=BROKER_PORT
        )

        print("Published to MQTT:")
        print(json.dumps(payload, indent=4))

        return True

    except Exception as e:
        print(f"Failed to publish command: {e}")
        return False
