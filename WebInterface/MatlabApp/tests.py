import json

from django.contrib.auth import get_user_model
from django.test import TestCase

import mqtt_subscriber
from MatlabApp.models import Message

TELEMETRY = '{"time": "0.100", "position": "0.0000", "angle": "3.1416", "control": "0.000"}'


class TelemetrySaveTests(TestCase):
    """mqtt_subscriber must store each reading against the user who started the run."""

    def setUp(self):
        self.user = get_user_model().objects.create_user(username="student", password="pw")

    def start(self, **extra):
        mqtt_subscriber.handle_command(json.dumps({"command": "sta", "experiment": "SwingHoldPendulum", **extra}))

    def test_reading_is_saved_against_the_user_who_started_the_run(self):
        self.start(user_id=self.user.id)
        mqtt_subscriber.handle_telemetry(TELEMETRY)
        msg = Message.objects.get()
        self.assertEqual(msg.user, self.user)
        self.assertEqual(msg.experiment, "SwingHoldPendulum")
        self.assertEqual(msg.message["angle"], "3.1416")

    def test_reading_from_a_run_started_outside_the_website_is_still_saved(self):
        self.start()
        mqtt_subscriber.handle_telemetry(TELEMETRY)
        self.assertIsNone(Message.objects.get().user)
