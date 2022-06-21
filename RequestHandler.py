import datetime
import json

import requests


def get_current_data(timeout=1):
    url = "http://bus.hwhhome.net:8080/bus"

    try:
        response = requests.post(
            url,
            headers={'Content-Type': 'application/json'},
            json={'search': 'SKK'},
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

def delete_mutiple(datas):
    sorted_datas = sorted(datas, key=lambda x: (x.sensor_id, x.timeline), reverse=True)

    result = []
    last = -1
    for s_d in sorted_datas:
        if last != s_d.sensor_id:
            result.append(s_d)
            last = s_d.sensor_id
    return result




class SensorData:
    co2: int
    sensor_id: str
    timeline: datetime.datetime

    @staticmethod
    def from_json(json_data):
        sensor_data = SensorData()
        sensor_data.sensor_id = str(json_data['sensorid'])
        sensor_data.timeline = datetime.datetime.strptime(json_data['timeline'], '%Y-%m-%d %H:%M:%S')
        try:
            sensor_data.co2 = float(json_data['senco2'])
        except:
            # Noneだとどっかしらで止まるといけないので
            sensor_data.co2 = 0

        return sensor_data

    def __str__(self):
        return f"{self.sensor_id} in {self.timeline}:{self.co2}"