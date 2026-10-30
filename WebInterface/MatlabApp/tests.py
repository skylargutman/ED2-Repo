import json

import re

from django.contrib.auth import get_user_model
from django.contrib.staticfiles import finders
from django.test import TestCase
from django.urls import reverse

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


class PageScriptTests(TestCase):
    """Pages must load JavaScript files that exist, with the URLs filled in by the template."""

    def setUp(self):
        user = get_user_model().objects.create_user(username="student", password="pw")
        self.client.force_login(user)

    def scripts(self, html):
        return re.findall(r'<script src\s*=\s*"/static/([^"]+)"', html)

    def assert_scripts_exist(self, url):
        html = self.client.get(url).content.decode()
        scripts = self.scripts(html)
        self.assertTrue(scripts, f"no local scripts on {url}")
        for path in scripts:
            found = finders.find(path)
            self.assertIsNotNone(found, f"{url} loads /static/{path}, which does not exist")
            with open(found, encoding="utf-8") as f:
                self.assertNotIn("{%", f.read(), f"{path} has a template tag, which only works in a template")
        return html

    def test_experiment_page(self):
        html = self.assert_scripts_exist(reverse("experiment_run_dynamic", args=["SwingHoldPendulum"]))
        self.assertIn(f'command: "{reverse("send_experiment_command", args=["SwingHoldPendulum"])}"', html)
        self.assertIn(f'updateParams: "{reverse("update_experiment_params", args=["SwingHoldPendulum"])}"', html)

    def test_live_stream_page(self):
        self.assert_scripts_exist(reverse("pendulum_stream"))
