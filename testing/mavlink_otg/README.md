# Minimal MAVLink CDC ACM OTG test

Pure NuttX target for the ESP32-S3 OTG request-lifecycle test.

It implements only MAVLink v2 HEARTBEAT, PARAM_REQUEST_LIST/PARAM_VALUE,
and PARAM_SET ACK. Run from NSH:

    mavlink_otg &

Host:

    python3 Tools/otg_mini_qgc.py --usbdev <bus/device> -n 20
