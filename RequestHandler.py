import datetime
import json

import requests


def get_current_data(timeout=1, new_api=False, search="SKK"):
    url = "http://bus.hwhhome.net:8080/bus" if not new_api else "http://bus.hwhhome.net/request_sensing"
    json_req = {"SEARCH_BUS_NUMBER": "EX_HADANO"}
    try:
        response = requests.post(
            url,
            headers={'Content-Type': 'application/json'},
            # json={('search' if not new_api else "SEARCH_BUS_NUMBER"): search}
            json=json_req,
            timeout=timeout
        )

        return response.json()
    except requests.exceptions.Timeout:
        return -1


def convert_jsons(jsons, time_th=-1, new_api=False):
    raw_datas = jsons['data']
    datas = list(map(lambda x: SensorData.from_json(x, new_api), raw_datas))

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
    def from_json(json_data, new_api=False):
        if not new_api:
            idx_id = "sensorid"
            idx_co2 = "senco2"
            idx_time = "timeline"
        else:
            idx_id = "IDENTIFIER"
            idx_co2 = "CO2"
            idx_time = "DATETIME"

        sensor_data = SensorData()
        sensor_data.sensor_id = str(json_data[idx_id])
        sensor_data.timeline = datetime.datetime.strptime(json_data[idx_time], '%Y-%m-%d %H:%M:%S')
        try:
            sensor_data.co2 = float(json_data[idx_co2])
        except:
            # Noneだとどっかしらで止まるといけないので
            sensor_data.co2 = 0

        return sensor_data

    def __str__(self):
        return f"{self.sensor_id} in {self.timeline}:{self.co2}"