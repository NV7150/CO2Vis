import asyncio

from VisualizeServer import up_server

async def main():
    await up_server(
        "sampleData/bus.ply",
        "sampleData/bus2/textured_output.obj",
        "sampleData/bus2",
        voxel=0.025,
        reflesh_rate=5,
        save_file="sampleData/bus2.json",
        # new_api=True,
        # search_bus="EX_HADANO",
        random_data=True,
        ws_port=8754,
        sample=-1,
    )

if __name__ == "__main__":
    asyncio.run(main())
