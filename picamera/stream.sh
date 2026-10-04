#!/bin/bash
# stream.sh v3.0

SERVER="sciencelabtoyou.com"
STREAM_NAME="pendulum"
STREAM_USER=${STREAM_USER:user}
STREAM_PASSWORD=${STREAM_PASSWORD:password}

echo "Starting stream to $SERVER..."

while true; do
    rpicam-vid -t 0 \
        --width 640 --height 480 \
        --framerate 30 \
        --intra 15 \
        --bitrate 2000000 \
        --codec h264 --inline \
        --nopreview -o - | \
    ffmpeg \
        -fflags nobuffer \
        -flags low_delay \
        -probesize 32 \
        -analyzeduration 0 \
        -i - \
        -c:v copy \
        -f mpegts \
        "srt://$SERVER:8890?streamid=publish:$STREAM_NAME:$STREAM_USER:$STREAM_PASSWORD&pkt_size=1316&latency=50000"

    echo "Stream dropped, reconnecting in 5s..."
    sleep 5
done
