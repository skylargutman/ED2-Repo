# Requirements

* Note from Claude:
    * ```
      Derived from the package manifest of the venv that ran on the previous Oracle instance (Python 3.9 / Django 4.2 LTS),
      which is the only reliable record of what production actually used. The old requirements.txt was UTF-16 encoded and
      listed Django at both 4.2.26 and 5.2.7; settings.py's header claims 5.2.7, but Django 5.2 cannot run on Python 3.9, so
      4.2 LTS is what was really deployed.

      Django 4.2 LTS supports Python 3.8-3.12, so this file is valid on the new Ubuntu 24.04 target (Python 3.12).
      ```
* Install Guide:
    * Get uv ([guide](https://docs.astral.sh/uv/getting-started/installation/))
    * Run `uv sync`

## Core web stack

* Django
* asgiref
* sqlparse

## ASGI serving

* Note from Claude:
    * ```
      (Channels needs ASGI; gunicorn runs a uvicorn worker)
      ```
* gunicorn
* uvicorn[standard]

## WebSockets / live sensor feed

* channels
* channels-redis
* redis
* msgpack

## MQTT bridge to the control Pi

* Note from Claude:
    * ```
      # mqtt_subscriber.py explicitly requests the 2.x VERSION2 callback API, matching
      # dac_daemon.py on the Pi. Do not downgrade below 2.0 without also reverting
      # those callback signatures to the v1 shape.
      ```

* paho-mqtt

## Database

* Note from Claude:
    * ```
      # settings.py calls pymysql.install_as_MySQLdb () to back the mysql engine. 
      ```
* PyMySQL

## Misc.

* django-cors-headers
* python-dotenv
    * Claude: "loads `deploy/.env` so secrets stay out of git"
