import sys, time, cv2, numpy as np # Default Python Libraries
path: str = sys.argv[1]; sys.path.insert(0, path) # Modifying Python Path

from scheduler.config import * # Configuration
from scheduler.client import Client as SClient # Scheduler Client
from scheduler.utils import Message, printc # Utilities

from client.client import Client as OClient

if __name__ == '__main__':
    # Creating clients
    object_client: OClient = OClient(disp=True, prnt=False)
    scheduler_client: SClient = SClient('facial')
    scheduler_client.connect()

    while True:
        # Asking ZMQ For Next Task
        time.sleep(STALL)
        response1: Message = scheduler_client.communicate('get', 'null')

        # Getting The Task
        split: list[str] = response1.data.split(' ')
        command, args = split[0], split[1:]
        if (command == DEFAULT_RESP): continue
        if (command == 'exit' or command == 'quit'): break

        # Running The Task
        try:
            task = object_client.interpret_task(command)
            object_client.image = scheduler_client.image()
            names = task(args)
            print(names)
        except:
            names = []

        # Sending The Result Back To ZMQ
        if names:
            response2: Message = scheduler_client.communicate('put', str(list(names)))

    # Closing client
    scheduler_client.close()
    printc('Program terminated.', INF_COLOR)