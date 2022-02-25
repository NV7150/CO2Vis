from threading import Lock


class Locker:
    def __init__(self):
        self.lock = Lock()

    def __enter__(self):
        self.lock.acquire()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.lock.release()
