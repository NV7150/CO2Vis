import asyncio
import datetime
import threading

from RequestHandler import SensorData
from VisualizeServer import up_server
from resourceController import ResourceController, requesting
from VirtualCo2Server import VirtualServer

async def main(resource):
    await up_server(
        "sampleData/bus.ply",
        "sampleData/bus2/textured_output.obj",
        "sampleData/bus2",
        voxel=0.025,
        reflesh_rate=5,
        save_file="sampleData/bus2-2.rev.json",
        new_api=True,
        # search_bus="EX_HADANO",
        # random_data=True,
        ws_port=8754,
        sample=-1,
        resource_con=resource
    )

async def main_virutal(filename, start_time, timescale):
    sensor_datas = []
    with open(filename, encoding="utf-8") as f:
        ds = f.read()
        ds = ds.split("\n")
        for d in ds[1:]:
            d = d.replace('"', "")
            d_raw = d.split(",")
            if len(d_raw) != 8:
                continue
            s_d = SensorData()
            s_d.co2 = d_raw[4]
            s_d.sensor_id = d_raw[0]
            s_d.timeline = datetime.datetime.strptime(d_raw[1], '%Y-%m-%d %H:%M:%S')
            sensor_datas.append(s_d)

    virutal_server = VirtualServer(datetime.datetime.strptime(start_time, '%Y-%m-%d %H:%M:%S'), sensor_datas, timescale)
    await up_server(
        "sampleData/bus.ply",
        "sampleData/bus2/textured_output.obj",
        "sampleData/bus2",
        voxel=0.05,
        reflesh_rate=0.0001,
        save_file="sampleData/bus2-2.rev.json",
        new_api=False,
        # search_bus="EX_HADANO",
        # random_data=True,
        ws_port=8754,
        sample=-1,
        virtual_server=virutal_server
    )


def up_normal():
    resource = ResourceController()

    def req():
        requesting(resource)

    th = threading.Thread(target=req)

    th.start()
    asyncio.run(main(resource))
    th.join()
    # asyncio.run(requesting(resource, 10))

def up_virtual():
    asyncio.run(main_virutal("sampleData/2022-03-02-CO2-HADANO-EX.csv", "2023-03-01 13:23:00", 60))

if __name__ == "__main__":
    up_virtual()