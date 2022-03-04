import datetime
import json

import requests


def get_current_data(timeout=1):
    url = "http://bus.hwhhome.net:8080/bus"

    try:
        response = requests.post(
            url,
            headers={'Content-Type': 'application/json'},
            json={'search': 'SHOUNAN330A7040'},
            timeout=timeout
        )

        return response.json()
    except requests.exceptions.Timeout:
        return -1


def convert_jsons(jsons, time_th=-1):
    raw_datas = jsons['data']
    datas = list(map(SensorData.from_json, raw_datas))

    if time_th == -1:
        return datas

    now = datetime.datetime.now()

    datas = [data for data in datas if (now - data.timeline).total_seconds() / 60 < time_th]

    return datas


class SensorData:
    co2: int
    sensor_id: str
    timeline: datetime.datetime

    @staticmethod
    def from_json(json_data):
        sensor_data = SensorData()
        # 下４桁に固定
        sensor_data.sensor_id = str(json_data['sensorid'])[-4:]
        sensor_data.timeline = datetime.datetime.strptime(json_data['timeline'], '%Y-%m-%d %H:%M:%S')
        sensor_data.co2 = int(json_data['senco2'])

        return sensor_data
