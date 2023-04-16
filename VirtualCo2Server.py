import datetime
from collections import defaultdict

from RequestHandler import SensorData


class VirtualServer:
    def __init__(self, start_time, sensor_datas, timescale=1.0):
        self.timescale = timescale
        self.curr_time = start_time

        datas = {}
        for s_data in sensor_datas:
            if s_data.sensor_id not in datas.keys():
                datas.setdefault(s_data.sensor_id, [])
            datas[s_data.sensor_id].append(s_data)

        sorted_datas = {}
        for key, s_datas in datas.items():
            sorted_sdatas = sorted(s_datas, key=lambda x: x.timeline)
            sorted_datas.setdefault(key, sorted_sdatas)
        self.timeline_data = sorted_datas
        self.curr_data = {key: 0 for key in self.timeline_data.keys()}

    def next_data(self):
        print(f"printing: {self.curr_time.strftime('%Y-%m-%d %H:%M:%S')}")

        # curr_data + 1 が curr_time よりも大きくなるまでインクリメントする
        for key in self.curr_data.keys():

            while self.curr_data[key] + 1 < len(self.timeline_data[key]) and self.curr_time > self.timeline_data[key][self.curr_data[key] + 1].timeline:
                self.curr_data[key] += 1

        returns = [self.timeline_data[key][i] for key, i in self.curr_data.items()]
        self.curr_time += datetime.timedelta(seconds=self.timescale)

        # curr_timeをまたぐ二つのdataの時間軸を比較して，近い方を採用
        # for i, key in enumerate(self.curr_data.keys()):
        #     curr_time = self.timeline_data[key][self.curr_data[key]].timeline
        #     next_time = self.timeline_data[key][self.curr_data[key] + 1].timeline
        #
        #     if abs((curr_time - self.curr_time).total_seconds()) < abs((next_time - self.curr_time).total_seconds()):
        #         self.curr_data[key] += 1

        return_dict = {"data": [
            {
                "sensorid": p.sensor_id,
                "timeline": p.timeline.strftime('%Y-%m-%d %H:%M:%S'),
                "senco2": p.co2
            }
            for p in returns
        ]}

        return return_dict








