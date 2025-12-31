package sensor

type IMUDataType struct {
	ID        uint32
	Acc       [3]float32
	Gyro      [3]float32
	Mag       [3]float32
	Euler     [3]float32
	Quat      [4]float32
	Pressure  float32
	Timestamp uint32
}

type IMUDataTypeWrapped struct {
	IMUDataType
	ID       string
	Seq      uint64
	SysTicks int64
}

type Sensor interface {
	Read() ([]IMUDataTypeWrapped, error)
	Reset() error
	Close() error
	Open() error
	ID() string
	Seq() uint64
}
