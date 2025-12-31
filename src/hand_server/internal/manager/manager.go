package manager

import "hand_apiserver/internal/sensor"

type Manager interface {
	Start() error
	Stop() error
	Restart() error
	Read(int64) (int64, []map[string]*sensor.IMUDataTypeWrapped, error)
	Running() bool
	ManuallyStopped() bool
	Faulted() bool
	ListDev() ([]string, error)
	ProbeDev() ([]string, error)
	TrySleep() error
}
