import random
import time
import json
from datetime import date

# Using the Python Device SDK for IoT Hub:
#   https://github.com/Azure/azure-iot-sdk-python
# The sample connects to a device-specific MQTT endpoint on your IoT Hub.
from azure.iot.device import IoTHubDeviceClient, Message

# The device connection string to authenticate the device with your IoT hub.
# Using the Azure CLI:
# az iot hub device-identity show-connection-string --hub-name {YourIoTHubName} --device-id MyNodeDevice --output table
CONNECTION_STRING = "HostName=AlphaTeam.azure-devices.net;DeviceId=mypi;SharedAccessKey=72t0bN7i5epLOIb8tnmLk6mFbCHhngu75Eg6ISN52Kc="


def iothub_client_init():
    # Create an IoT Hub client
    client = IoTHubDeviceClient.create_from_connection_string(CONNECTION_STRING)
    return client

def iothub_client_message():

    try:
        client = iothub_client_init()
        print ( "IoT Hub device sending periodic messages, press Ctrl-C to exit" )
        while True:
            # Build the message
            today = str(date.today())
            now = time.localtime()
            current_time = str(time.strftime("%H:%M:%S"))
            location = "Row 7 Bin 4"
            msg = {
                "Date": today,
                "Time": current_time,
                "Location": location
                }
            msg_s = json.dumps(msg)

            # Send the message.
            print( "Sending message: {}".format(msg_s) )
            client.send_message(msg_s)
            print ( "Message successfully sent" )
            time.sleep(10)

    except KeyboardInterrupt:
        print ( "IoTHubClient sample stopped" )

if __name__ == '__main__':
    print ( "IoT Hub Quickstart #1 - Simulated device" )
    print ( "Press Ctrl-C to exit" )
    iothub_client_message()
