import json
from channels.generic.websocket import AsyncWebsocketConsumer


class SensorDataConsumer(AsyncWebsocketConsumer):
    """Fans out everything the MQTT bridge receives to open dashboards.

    Two kinds of message travel over this socket, distinguished client-side by
    the top-level `type` field:

      sensor_update -> {"data": {...}}                  live chart values
      rig_status    -> {"type": "rig_status", ...}      rig state badge

    sensor_update deliberately keeps its original wire format (a bare payload
    under `data`) so existing client code continues to work unchanged.
    """

    async def connect(self):
        self.group_name = 'sensor_data'
        await self.channel_layer.group_add(
            self.group_name,
            self.channel_name
        )
        await self.accept()

    async def disconnect(self, close_code):
        await self.channel_layer.group_discard(
            self.group_name,
            self.channel_name
        )

    async def sensor_update(self, event):
        await self.send(text_data=json.dumps(event['data']))

    async def rig_status(self, event):
        await self.send(text_data=json.dumps({
            'type': 'rig_status',
            'status': event.get('status', 'unknown'),
            'detail': event.get('detail', ''),
        }))
