
import Queue

class request:
    def __init__(self, device, pack):
        self.device = device
        self.txBuffer = pack   


reqQueue = Queue.PriorityQueue()


newReq = request("Req 1", "pack")
reqQueue.put((2, newReq))

newReq = request("Req 2", "pack")
reqQueue.put((2, newReq))

newReq = request("Req 3", "pack")
reqQueue.put((1, newReq))


while reqQueue.qsize() > 0:
    currentRequest = reqQueue.get()[1]
    print currentRequest.device
