#!/usr/bin/env python3
"""Get data from TTN
"""

import base64
import json
import struct

import paho.mqtt.client as mqtt
from influxdb import InfluxDBClient

# Change with values from your TTN Application
TTN_USERNAME = 'iottrack'
TTN_PASSWORD = 'ttn-account-v2.0000000000000000000000000000000000000000000'
TTN_BROKER = 'eu.thethings.network'
TTN_PORT = 1883
TTN_SUBSCRIPTIONS = '+/devices/+/up'


# Setup your InfluxDB server and create the data table
INFLUX_HOST = 'localhost'
INFLUX_DB = 'iottrack_lora'


class IoTTrackLoRa:
    def __init__(self, mqtt_client, influx_client):
        self.mqtt_client = mqtt_client
        self.influx_client = influx_client

        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.username_pw_set(TTN_USERNAME, TTN_PASSWORD)

        self.mqtt_client.connect(TTN_BROKER, TTN_PORT, 60)

    def run(self):
        self.mqtt_client.loop_forever()

    @staticmethod
    def on_connect(client, userdata, flags, return_code):
        """Callback function for MQTT connection event
        """

        print('Connected with result code {}'.format(return_code))

        client.subscribe(TTN_SUBSCRIPTIONS)

    def on_message(self, client, userdata, msg):
        """Callback function for MQTT new message event
        """
        data = json.loads(msg.payload.decode())
        payload_raw = data['payload_raw']
        payload = base64.b64decode(payload_raw)
        version, battery, temperature, pressure = struct.unpack('>BHhH', payload)

        print('\ntopic: {}\nmessage: {}'.format(msg.topic, payload))
        print('\tVersion: {}\n\tBattery: {}V\n\tTemperature: {}ºC\n\tPressure: {}hPa'.format(
            version, battery / 1000, temperature / 10, pressure / 10))

        json_body = [
            {
                "measurement": "environment",
                "fields": {
                    "battery": battery / 1000,
                    "temperature": temperature / 10,
                    "pressure": pressure / 10
                },
                "tags": {
                    "dev_id": data['dev_id'],
                    "pkt_version": version
                }
            }
        ]

        self.influx_client.write_points(json_body)


def main():
    """Main Loop
    """

    client = mqtt.Client()
    influx = InfluxDBClient(host=INFLUX_HOST, database=INFLUX_DB)

    iottrack_lora = IoTTrackLoRa(mqtt_client=client, influx_client=influx)

    iottrack_lora.run()


if __name__ == "__main__":
    main()
