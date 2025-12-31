package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"time"

	"github.com/tarm/serial"
)

var logger = log.New(os.Stdout, "hi229", log.LstdFlags)

// Function to construct the message sequence based on fps and mode
func getMsgs(fps int, mode string) ([][]byte, error) {
	var modeVal string
	if mode == "6dof" {
		modeVal = "0"
	} else if mode == "9dof" {
		modeVal = "1"
	} else {
		return nil, fmt.Errorf("invalid mode")
	}

	msgs := [][]byte{
		[]byte("AT+EOUT=0\r\n"),
		[]byte(fmt.Sprintf("AT+MODE=%s\r\n", modeVal)),
		[]byte(fmt.Sprintf("AT+SETPTL=91\r\n")),
		[]byte(fmt.Sprintf("AT+ODR=%d\r\n", fps)),
	}

	if fps == 200 {
		msgs = append(msgs[:3], []byte("AT+BAUD=921600\r\n"))
	}

	msgs = append(msgs, []byte("AT+EOUT=1\r\n"), []byte("AT+RST\r\n"))

	return msgs, nil
}

func configure(port string, fps int, mode string, baud int) error {
	msgs, err := getMsgs(fps, mode)
	if err != nil {
		return err
	}

	c := &serial.Config{Name: port, Baud: baud}
	s, err := serial.OpenPort(c)
	if err != nil {
		return err
	}
	defer s.Close()

	for _, msg := range msgs {
		_, err := s.Write([]byte(msg))
		if err != nil {
			return err
		}
		time.Sleep(2 * time.Second)

		buf := make([]byte, 128)
		n, err := s.Read(buf)
		if err != nil {
			return err
		}

		fmt.Printf("host -> %simu  -> %s", msg, msg, buf[:n])
	}

	return nil
}

func main() {
	portFlag := flag.String("port", "", "The serial port to use")
	fpsFlag := flag.Int("fps", 100, "Frequency in Hz (100 or 200)")
	modeFlag := flag.String("mode", "9dof", "Mode (6dof or 9dof)")
	baudFlag := flag.Int("baud", 115200, "Baud rate")

	flag.Parse()

	if *portFlag == "" || (*fpsFlag != 100 && *fpsFlag != 200) || (*modeFlag != "6dof" && *modeFlag != "9dof") {
		logger.Fatal("--port must be specified along with valid --fps (100 or 200) and --mode (6dof or 9dof)")
	}

	err := configure(*portFlag, *fpsFlag, *modeFlag, *baudFlag)
	if err != nil {
		logger.Fatal(err)
	}
}
