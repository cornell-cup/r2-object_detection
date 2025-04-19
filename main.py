import time, os # Default Python Libraries

from client.client import * # Importing The Client/Task Manager

# Global Variables (Sleep Time For Ability To See Results, Is Not Needed)
SLEEP_TIME: float = 0.25

if __name__ == '__main__':
    client: Client = Client(disp=True, prnt=True, open=True) # Creating The Client

    while True:
        # Clearing Terminal & Getting Inputs
        os.system('cls' if os.name == 'nt' else 'clear')
        inputs = input().split(' ')
        command, args = inputs[0].lower(), inputs[1:]

        # Parsing Commands, Running Tasks, & Sleeping For Ability To See Results
        if (command == 'exit' or command == 'e'): break
        if (command == 'quit' or command == 'q'): break

        task = client.interpret_task(command)
        result = task(args)
        time.sleep(SLEEP_TIME)