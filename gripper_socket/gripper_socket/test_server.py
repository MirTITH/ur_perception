import socket
import json


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("localhost", 8848))
    server.listen(5)
    print("Server started")
    while True:
        conn, addr = server.accept()
        print("Connected by", addr)
        while True:
            raw_data = conn.recv(1024)
            data: dict = json.loads(raw_data)
            print(f"Received size: {len(raw_data)}. Data: {data}")
            if not raw_data:
                break
        conn.close()
        print("Connection closed")


if __name__ == "__main__":
    main()
