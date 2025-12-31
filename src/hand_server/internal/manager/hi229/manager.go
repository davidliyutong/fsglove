package hi229

import (
	"container/list"
	"context"
	"errors"
	"fmt"
	log "github.com/sirupsen/logrus"
	"github.com/tarm/serial"
	"hand_apiserver/internal/config"
	"hand_apiserver/internal/manager"
	"hand_apiserver/internal/sensor"
	"hand_apiserver/internal/sensor/hi229"
	"math"
	"os"
	"runtime"
	"strings"
	"sync"
	"time"
)

const BufLen = 1024

type hi229Manager struct {
	opt              *config.RFMoCapOpt
	sensors          map[string]sensor.Sensor
	ringBuffer       []map[string]*sensor.IMUDataTypeWrapped
	ctx              context.Context
	cancel           context.CancelFunc
	wg               sync.WaitGroup
	lock             sync.RWMutex
	running          bool
	counter          int64
	manuallyStopped  bool
	faulted          bool
	lastAccessSecond int64
}

func listSerialPorts() []string {
	// This function lists serial ports depending on the platform
	var ports []string
	switch runtime.GOOS {
	case "windows":
		for i := 1; i <= 256; i++ {
			ports = append(ports, fmt.Sprintf("COM%d", i))
		}
	case "linux":
		// On Linux, serial ports are usually named /dev/ttyS* or /dev/ttyUSB*
		files, err := os.ReadDir("/dev")
		if err != nil {
			log.Errorln("Error reading directory:", err)
		}

		// Filter and add matching files to the slice
		for _, file := range files {
			if strings.Contains(file.Name(), "tty") && strings.Contains(file.Name(), "USB") {
				ports = append(ports, fmt.Sprintf("/dev/%s", file.Name()))
			}
		}
	case "darwin":
		// On MacOS, serial ports are usually named /dev/tty.*
		files, err := os.ReadDir("/dev")
		if err != nil {
			log.Fatal(err)
		}
		for _, file := range files {
			if file.IsDir() {
				continue
			}
			name := file.Name()
			if len(name) > 4 && name[:4] == "tty." {
				ports = append(ports, "/dev/"+name)
			}
		}
	default:
		log.Fatalf("unsupported platform: %s", runtime.GOOS)
	}
	return ports
}

func testPort(portName string) bool {
	c := &serial.Config{Name: portName, Baud: hi229.DefaultBaudRate, ReadTimeout: time.Second * 5}
	s, err := serial.OpenPort(c)
	if err != nil {
		return false
	}
	fmt.Print(".")
	time.Sleep(time.Millisecond * 100)

	defer func(s *serial.Port) {
		err := s.Close()
		if err != nil {
		}
	}(s)

	buffer := make([]byte, 4096)
	n, err := s.Read(buffer)
	if n > 0 {
		return true
	} else {
		return false
	}
}

func (m *hi229Manager) ProbeDev() ([]string, error) {
	ports := listSerialPorts()
	var validPorts []string

	for _, portName := range ports {
		if testPort(portName) {
			validPorts = append(validPorts, portName)
		}
	}

	if len(validPorts) == 0 {
		return nil, errors.New("no valid ports found")
	} else {
		return validPorts, nil
	}
}

const autoSleepDurationSecond = 60

func (m *hi229Manager) TrySleep() error {
	var err error = nil
	if m.Running() && (time.Now().Unix()-m.lastAccessSecond > autoSleepDurationSecond) {
		log.Infof("timeout after %v seconds, enter sleep mode", autoSleepDurationSecond)
		m.lastAccessSecond = math.MaxInt64
		err := m.Stop()
		if err != nil {
			log.Errorln(err)
		}
	}
	return err
}

// ListDev returns a list of sensors
func (m *hi229Manager) ListDev() ([]string, error) {
	m.lastAccessSecond = time.Now().Unix()

	res := make([]string, len(m.sensors))
	idx := 0
	for _, s := range m.sensors {
		res[idx] = s.ID()
		idx++
	}
	return res, nil
}

func (m *hi229Manager) Running() bool {
	return m.sensors != nil && !m.faulted
}

func (m *hi229Manager) Faulted() bool {
	return m.faulted
}

func (m *hi229Manager) ManuallyStopped() bool {
	return m.manuallyStopped
}

const TimestampThresholdNanoSecond = 20000000
const minCacheSize = 8
const minStopSize = 2

func (m *hi229Manager) updateAll() {
	// create cache queues
	cache := make(map[string]*list.List, len(m.sensors))
	for _, rs := range m.sensors {
		cache[rs.ID()] = list.New()
	}
	// create flag to control the flow
	var triggered = false

	// diagnose variables
	diagLastCheck := time.Now().UnixMilli()
	diagLastCounter := m.counter
	diagPCounter := 0

	for {
	start:
		select {
		case <-m.ctx.Done():
			m.wg.Done()
			return
		default:
		}

		for _, rs := range m.sensors {
			res, err := rs.Read()
			diagPCounter += len(res)
			if err != nil {
				log.Debugf("sensor %v error: %v", rs.ID(), err)
				if err.Error() == "EOF" {
					m.faulted = true
					return
				} else {
					continue
				}
			}

			for _, item := range res {
				cache[rs.ID()].PushBack(&item)
			}
		}

		diagDuration := float64(time.Now().UnixMilli()-diagLastCheck) / 1000
		if diagDuration >= 10 {
			log.Debugf("updateAll fps: %3.1f, pps: %3.1f", float64(m.counter-diagLastCounter)/diagDuration, float64(diagPCounter)/diagDuration)
			diagLastCounter = m.counter
			diagLastCheck = time.Now().UnixMilli()
			diagPCounter = 0
		}

		for {
			for deviceID, _ := range cache {
				if !triggered && cache[deviceID].Len() < minCacheSize {
					goto start
				} else if triggered && cache[deviceID].Len() < minStopSize {
					triggered = false
					goto start
				}
			}
			triggered = true

			candidates := make(map[string]*sensor.IMUDataTypeWrapped)
			newestTimestamp := int64(0)
			newestDeviceID := ""

			for deviceID, _ := range cache {
				if cache[deviceID].Len() > 0 {

					candidates[deviceID] = cache[deviceID].Front().Value.(*sensor.IMUDataTypeWrapped)

					if candidates[deviceID].SysTicks > newestTimestamp+TimestampThresholdNanoSecond/2 {
						newestTimestamp = candidates[deviceID].SysTicks
						newestDeviceID = deviceID
						cache[deviceID].Remove(cache[deviceID].Front())

						for otherDeviceID, _ := range candidates {
							if otherDeviceID != newestDeviceID {
								for {
									if cache[otherDeviceID].Len() <= 0 {
										break
									}
									packet := cache[otherDeviceID].Front().Value.(*sensor.IMUDataTypeWrapped)
									if packet.SysTicks > newestTimestamp-TimestampThresholdNanoSecond/2 {
										candidates[otherDeviceID] = packet
										break
									} else {
										cache[otherDeviceID].Remove(cache[otherDeviceID].Front())
									}
								}
							}
						}

					} else if candidates[deviceID].SysTicks > newestTimestamp-TimestampThresholdNanoSecond/2 {
						cache[deviceID].Remove(cache[deviceID].Front())
						continue
					} else {
						for {
							if cache[deviceID].Len() <= 0 {
								delete(candidates, deviceID)
								break
							}

							packet := cache[deviceID].Front().Value.(*sensor.IMUDataTypeWrapped)
							cache[deviceID].Remove(cache[deviceID].Front())
							if packet.SysTicks > newestTimestamp-TimestampThresholdNanoSecond/2 {
								candidates[deviceID] = packet
								break
							}
						}
					}

				} else {
					triggered = false
				}
			}
			if len(candidates) == len(m.sensors) {
				m.ringBuffer[m.counter%BufLen] = candidates
				m.counter++
			} else {
				log.Warnln("updateAll encounter a sync failure")
			}
		}
	}
}

// Start starts the sensor manager
func (m *hi229Manager) Start() error {
	m.lock.Lock()
	defer m.lock.Unlock()
	m.lastAccessSecond = time.Now().Unix()
	log.Infof("manager started")

	if m.sensors == nil {
		m.ctx, m.cancel = context.WithCancel(context.Background())
		m.sensors = make(map[string]sensor.Sensor)
		faulted := true
		for _, imu := range m.opt.IMU {
			if imu.ID == "" {
				return errors.New("empty sensor id")
			}
			if imu.Name == "" {
				return errors.New("empty sensor name")
			}
			if _, ok := m.sensors[imu.ID]; ok {
				return errors.New("duplicate sensor id:  " + imu.ID)
			}
			s := hi229.NewSensor(config.IMUOpt{Name: imu.Name, Baud: imu.Baud, ID: imu.ID})
			if s == nil {
				return errors.New("failed to create sensor: " + imu.Name)
			}
			m.sensors[imu.ID] = s
			time.Sleep(time.Millisecond * 50) //  wait for stable
		}
		m.wg.Add(1)
		faulted = false
		m.faulted = faulted
		go m.updateAll()

		time.Sleep(time.Millisecond * 50) //  wait for stable
	}
	m.manuallyStopped = false
	return nil
}

// Stop stops the sensor manager
func (m *hi229Manager) Stop() error {
	m.lock.Lock()
	defer m.lock.Unlock()
	m.lastAccessSecond = time.Now().Unix()
	log.Infof("manager stopped")

	if m.sensors == nil {
		return nil
	}
	m.cancel()
	m.wg.Wait()
	for _, rs := range m.sensors {
		err := rs.Close()
		if err != nil {
			return err
		}
	}
	m.sensors = nil
	m.manuallyStopped = true
	m.counter = 0
	m.ringBuffer = make([]map[string]*sensor.IMUDataTypeWrapped, BufLen)
	return nil
}

// Restart restarts the sensor manager
func (m *hi229Manager) Restart() error {
	err := m.Stop()
	if err != nil {
		return err
	}
	return m.Start()
}

// Read reads the latest packet from each sensor
func (m *hi229Manager) Read(cursor int64) (int64, []map[string]*sensor.IMUDataTypeWrapped, error) {
	m.lock.RLock()
	defer m.lock.RUnlock()
	m.lastAccessSecond = time.Now().Unix()

	if cursor < 0 {
		cursor = m.counter - 1
		if cursor < 0 {
			return cursor, nil, errors.New("not ready")
		}
		res := make([]map[string]*sensor.IMUDataTypeWrapped, 1)
		res[0] = m.ringBuffer[cursor%BufLen]
		return cursor, res, nil
	} else {
		res := make([]map[string]*sensor.IMUDataTypeWrapped, 0)
		// read all remaining packets from each sensor
		if cursor+1 >= m.counter {
			return cursor, nil, errors.New("no new data")
		}
		stop := m.counter
		if stop-cursor >= BufLen {
			cursor = m.counter - 1
		}
		for {
			if cursor >= stop {
				break
			}
			res = append(res, m.ringBuffer[(cursor)%BufLen])
			cursor++
		}

		if len(res) == 0 {
			return cursor, nil, errors.New("no new data")
		} else {
			return cursor, res, nil
		}
	}
}

func NewManager(opt *config.RFMoCapOpt) manager.Manager {
	return &hi229Manager{
		opt:              opt,
		sensors:          nil,
		ringBuffer:       make([]map[string]*sensor.IMUDataTypeWrapped, BufLen),
		ctx:              nil,
		cancel:           nil,
		wg:               sync.WaitGroup{},
		lock:             sync.RWMutex{},
		counter:          0,
		manuallyStopped:  false,
		faulted:          false,
		lastAccessSecond: time.Now().Unix(),
	}
}

func Daemon(m manager.Manager) {
	for {
		//
		if m.Faulted() {
			log.Infoln("status is faulted, stopping")
			err := m.Stop()
			if err != nil {
				log.Errorln(err)
			}
		}
		if !m.Running() && !m.ManuallyStopped() {
			err := m.Start()
			if err != nil {
				log.Errorln(err)
				time.Sleep(time.Second * 1)
				continue
			}
		}
		time.Sleep(time.Second * 1)
		_ = m.TrySleep()

	}
}
