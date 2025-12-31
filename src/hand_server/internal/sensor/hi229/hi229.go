package hi229

import (
	"errors"
	log "github.com/sirupsen/logrus"
	"github.com/tarm/serial"
	"hand_apiserver/internal/config"
	sensor2 "hand_apiserver/internal/sensor"
	"sync"
	"time"
)

const MaxReadNum = 4096
const BufferSize = 4096
const MaxIMUDataNum = 100
const DefaultBaudRate = 115200

// sensor cannot be accessed by two goroutines at the same time
type sensor struct {
	id   string
	opt  config.IMUOpt
	port *serial.Port
	raw  rawType
	buf  [BufferSize]byte
	seq  uint64
}

func (s *sensor) Seq() uint64 {
	return s.seq
}

func (s *sensor) ID() string {
	return s.id
}

// Close closes the serial port
func (s *sensor) Close() error {
	if s.port == nil {
		return nil
	}
	err := s.port.Close()
	if err != nil {
		return err
	}
	s.port = nil
	return nil
}

// Open opens the serial port
func (s *sensor) Open() error {
	if s.port != nil {
		return nil
	}
	c := &serial.Config{
		Name:        s.opt.Name, // Replace with your serial port name
		Baud:        s.opt.Baud, // Replace with your baud rate
		ReadTimeout: time.Second * 5,
	}
	port, err := serial.OpenPort(c)
	if err != nil {
		log.Warnln(err)
		return err
	}
	s.port = port
	s.seq = 0
	return s.port.Flush()
}

// Reset resets the serial port cache
func (s *sensor) Reset() error {
	return s.port.Flush()
}

var imuWrappedPool = sync.Pool{
	New: func() interface{} {
		b := sensor2.IMUDataTypeWrapped{}
		return &b
	},
}

// Read reads the serial port and returns the IMU data
func (s *sensor) Read() ([]sensor2.IMUDataTypeWrapped, error) {
	if s.port == nil {
		return nil, errors.New("port not open")
	}
	results := make([]sensor2.IMUDataTypeWrapped, 0, MaxIMUDataNum)
	count := 0
	for count < MaxReadNum {
		n, err := s.port.Read(s.buf[:])
		if err != nil {
			return nil, err
		}
		if n == 0 {
			return nil, errors.New("port cannot be read")
		}
		count += n
		for i := 0; i < n; {
			if chSerialInput(&s.raw, s.buf[i]) == 1 {
				wrapped := imuWrappedPool.Get().(*sensor2.IMUDataTypeWrapped)
				wrapped.IMUDataType = s.raw.imu[0]
				wrapped.ID = s.id
				wrapped.Seq = s.seq
				wrapped.SysTicks = time.Now().UnixNano()

				results = append(results, *wrapped)
				imuWrappedPool.Put(wrapped)

				s.seq++
			}
			i++
		}
		if len(results) > 0 {
			return results, nil
		}
	}
	return nil, errors.New("IMU_FAIL")
}

func NewSensor(opt config.IMUOpt) sensor2.Sensor {
	s := sensor{
		id:   opt.ID,
		opt:  opt,
		port: nil,
		raw:  rawType{},
		buf:  [BufferSize]byte{},
		seq:  0,
	}
	err := s.Open()
	if err != nil {
		log.Warnln(err)
		return nil
	}

	return &s
}
