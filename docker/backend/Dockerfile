FROM ubuntu

ENV IMG_HOME_DIR=/home/lab_instruments
ENV IMAG_APP_DIR=${IMG_HOME_DIR}/ED2-Repo
ENV UV_PROJECT_ENVIRONMENT=/home/lab_instruments/web-venv

RUN apt-get update && apt-get install -y git vim python3 python3-venv sudo 
RUN useradd -m -s /bin/bash lab_instruments
RUN usermod -aG sudo lab_instruments

USER lab_instruments
WORKDIR ${IMG_HOME_DIR}

RUN mkdir backend-app
RUN python3 -m venv web-venv
RUN cd backend-app
WORKDIR backend-app
COPY  --chown=lab_instruments:lab_instruments pyproject.toml uv.lock .
RUN ../web-venv/bin/pip install uv
RUN ../web-venv/bin/uv sync --frozen


CMD ["/home/lab_instruments/web-venv/bin/uv", "run", "python",  "manage.py", "runserver", "0.0.0.0:8000"]
