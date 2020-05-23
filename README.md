# Arduino Solar Monitor - Monitor your LoRaWan Solar gateway

## Intro
The purpose of this project is to monitor a LoRaWan / TheThingsNetwork solar gateway remotely with an Arduino platform. The project reports:
- Internal Temperature (with ATMega accuracy)
- Battery voltage
- Battery flow current
- Battery flow power
- Status of an outgoing usb 5V source

## Project details

You can find project details on [Author blog](https://www.disk91.com/2020/projects/iot/lorawan-solar-gateway-monitoring/)

## Main Hardware involved
- LoRa Radio Node board (or Arduino + rfm95)
- LifePO4 3.2V AA battery
- INA219 Voltage and Current sensor

## Library required
- MCCI LoRaWAN LMIC (by IBM, Matthis Kooijman...) >= 3.2.0
- Low-Power (by Rocket Scream Electronics) >= 1.6.0

## Schematic
- See [related blog post](https://www.disk91.com/2020/projects/iot/lorawan-solar-gateway-monitoring/)


## Setup
1. Get your APPEUI / DEVEUI / APPKEY from [TTN Console](https://console.thethingsnetwork.org)
2. Setup your APPEUI / DEVEUI / APPKEY in the key.h file
3. Configure the MyDevice Cayenne Integration on TTN Console
4. Add the device in [Mydevice Cayenne dashboard](https://cayenne.mydevices.com) as a Cayenne LPP device.

## Reported Metrics

| Channel | Data              | Unit |
|---------|-------------------|-----:|
| 1       | Board Temperature | °C   |
| 2       | Battery current   |  A   |
| 3       | Battery Voltage   |  V   |
| 4       | Board Voltage     |  V   |
| 5       | 5V output status  |  0/1 |
| 6       | Battery Power     |  W   |





