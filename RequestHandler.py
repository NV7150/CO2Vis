import datetime
import json

import requests
import pandas as pd
import numpy as np


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


def convert_csv_data(line):
    cols = line.replace('","', '\t').split("\t")
    # cols = np.array(pd.read_csv(line))
    co2 = int(float(cols[-1].strip().replace('"', "")))
    timeline = datetime.datetime.strptime(cols[1], '%Y-%m-%d %H:%M:%S')
    sensor_id = cols[2]
    return SensorData(co2, sensor_id, timeline)


class SensorData:
    def __init__(self, co2, sensor_id, timeline):
        self.co2 = co2
        self.sensor_id = sensor_id
        self.timeline = timeline

    @staticmethod
    def from_json(json_data):
        sensor_id = str(json_data['sensorid'])
        timeline = datetime.datetime.strptime(json_data['timeline'], '%Y-%m-%d %H:%M:%S')
        try:
            co2 = float(json_data['senco2'])
        except:
            # Noneだとどっかしらで止まるといけないので
            co2 = 0

        sensor_data = SensorData(co2, sensor_id, timeline)

        return sensor_data

    def __str__(self):
        return f"{self.sensor_id} in {self.timeline}:{self.co2}"