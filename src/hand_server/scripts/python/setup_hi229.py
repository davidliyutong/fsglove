from serial import Serial
import sys
import logging
import time

logger = logging.getLogger("hi229")

# 100Hz and 200Hz configuration
msgs = {
    100: [
        b"AT+EOUT=0\r\n",
        b"AT+MODE=1\r\n",
        b"AT+SETPTL=91\r\n",
        b"AT+ODR=100\r\n",
        b"AT+EOUT=1\r\n",
        b"AT+RST\r\n",
    ],
    200: [
        b"AT+EOUT=0\r\n",
        b"AT+MODE=1\r\n",
        b"AT+SETPTL=91\r\n",
        b"AT+BAUD=921600\r\n",
        b"AT+ODR=200\r\n",
        b"AT+EOUT=1\r\n" b"AT+RST\r\n",
    ],
}


def configure(port: str, freq: int=100):
    assert freq in msgs.keys(), "invalid freq"
    s = Serial(port=port, baudrate=115200)
    # send the configuration messages
    for msg in msgs[freq]:
        s.write(msg)
        time.sleep(2)
        print(f"host ->{msg}\nimu  ->{s.read_all()[:16]}")


if __name__ == "__main__":
    assert len(sys.argv) > 1, "port is not provided"
    port = sys.argv[1]
    configure(port)
