from django.conf import settings  # for CustomUser
from django.db import models
# Control lock model to manage which user has control of the experiment at any given time, with a timeout mechanism
from django.utils import timezone


class ControlLock(models.Model):
    session_key = models.CharField(max_length=40, unique=True)
    user = models.CharField(max_length=100, blank=True, default='')
    acquired_at = models.DateTimeField(auto_now_add=True)
    last_heartbeat = models.DateTimeField(auto_now_add=True)

    @classmethod
    def get_active(cls, timeout_seconds=30):
        """Returns the active lock if one exists and hasn't timed out."""
        cutoff = timezone.now() - timezone.timedelta(seconds=timeout_seconds)
        return cls.objects.filter(last_heartbeat__gte=cutoff).first()


# Store parameters each time the user updates them
class ExperimentSession(models.Model):
    experiment_name = models.CharField(max_length=100)
    parameters = models.JSONField()
    user = models.ForeignKey(
        settings.AUTH_USER_MODEL,
        on_delete=models.CASCADE,
        related_name='experiment_sessions'
    )
    updated_at = models.DateTimeField(auto_now=True)

    def __str__(self):
        return f"{self.experiment_name} parameters for {self.user.username}"


# command Model, stores commands sent from the site, timestamp, and user
class Command(models.Model):
    instrument = models.CharField(max_length=100, default="pendulum")
    experiment = models.CharField(max_length=100, default="")
    user = models.ForeignKey(settings.AUTH_USER_MODEL, on_delete=models.CASCADE, related_name='commands', null=True)

    command = models.CharField(max_length=255)
    timestamp = models.DateTimeField(auto_now_add=True)

    def __str__(self):
        return f"{self.command} (by {self.user.username})"


# Sensor Data model defined but not used in the current iteration of the site, will be better down the line with more readings
class SensorData(models.Model):
    name = models.CharField(max_length=50)
    value = models.FloatField()
    timestamp = models.DateTimeField(auto_now_add=True)
    user = models.ForeignKey(settings.AUTH_USER_MODEL, on_delete=models.CASCADE, related_name='sensor_data',
                             null=True, blank=True)

    def __str__(self):
        return f"{self.name}: {self.value} @ {self.timestamp}"


# Message model stores messages from MATLAB, timestamp, and the user
class Message(models.Model):
    message = models.TextField()
    timestamp = models.DateTimeField(auto_now_add=True)

    # user = models.ForeignKey(settings.AUTH_USER_MODEL, on_delete=models.CASCADE, related_name='messages')

    def __str__(self):
        return f"{self.message} @ {self.timestamp} (to {self.user.username})"
