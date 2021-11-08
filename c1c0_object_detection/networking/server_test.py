from .Server import Server

if __name__ == "__main__":
    computer = Server()
    while True:
        x = computer.receive_data()
        if x != "no data within listening time":
            print(x)
            # run rrt
            computer.send_update([0, 0, 0, 0, 0, 0])  # placeholder
            break