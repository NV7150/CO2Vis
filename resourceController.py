import asyncio
import time

from RequestHandler import get_current_data
from ThreadingTools import Locker


class ResourceController:
    def __init__(self):
        self.resource = -1
        self.locker = Locker()

    def put(self, resource):
        with self.locker:
            self.resource = resource

    def read(self):
        resource = -1
        with self.locker:
            resource = self.resource
        return resource


def requesting(resource, interval=5, ):
    print("aaaaa")
    while True:
        data = get_current_data(timeout=10, new_api=True)
        print("received ", data)
        resource.put(data)
        time.sleep(interval)
