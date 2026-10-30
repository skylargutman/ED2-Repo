# Settings for running the test suite locally without MySQL or Redis:
#   python manage.py test --settings=EGNSite.test_settings
from EGNSite.settings import *  # noqa: F401,F403

DATABASES = {'default': {'ENGINE': 'django.db.backends.sqlite3', 'NAME': ':memory:'}}
CHANNEL_LAYERS = {'default': {'BACKEND': 'channels.layers.InMemoryChannelLayer'}}
