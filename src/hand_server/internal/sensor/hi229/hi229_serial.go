package hi229

import (
	log "github.com/sirupsen/logrus"
	sensor2 "hand_apiserver/internal/sensor"
	"math"
)

const (
	ChSync1   = 0x5A // CHAOHE message sync code 1
	ChSync2   = 0xA5 // CHAOHE message sync code 2
	ChHdrSize = 0x06 // CHAOHE protocol header size

	MaxRawLen   = 512 // max raw frame long
	MaxNodeSize = 1   // max support node count

	// data items
	kItemID           = 0x90
	kItemAccRaw       = 0xA0
	kItemGyrRaw       = 0xB0
	kItemMagRaw       = 0xC0
	kItemRotationEul  = 0xD0
	kItemRotationQuat = 0xD1
	kItemPressure     = 0xF0
	KItemIMUSOL       = 0x91
	KItemGWSOL        = 0x62
)

type rawType struct {
	nByte     int
	len       int
	buf       [MaxRawLen]uint8
	gWID      uint8
	nIMU      uint8
	imu       [MaxNodeSize]sensor2.IMUDataType
	itemCode  [8]uint8
	nItemCode uint8
}

// chSerialInput decodes the serial input
func chSerialInput(raw *rawType, data uint8) int {
	if raw.nByte == 0 {
		if !syncCh(&raw.buf, data) {
			return 0
		}
		raw.nByte = 2
		return 0
	}

	raw.buf[raw.nByte] = data
	raw.nByte++

	if raw.nByte == ChHdrSize {
		if raw.len = int(U2(raw.buf[2:])); raw.len > (MaxRawLen - ChHdrSize) {
			raw.nByte = 0
			return -1
		}
	}

	if raw.nByte < (raw.len + ChHdrSize) {
		return 0
	}

	raw.nByte = 0
	return decodeCh(raw)
}

func syncCh(buf *[512]uint8, data uint8) bool {
	buf[0] = buf[1]
	buf[1] = data
	return buf[0] == ChSync1 && buf[1] == ChSync2
}

func decodeCh(raw *rawType) int {
	var crc uint16 = 0

	crc16Update(&crc, raw.buf[:4])
	crc16Update(&crc, raw.buf[6:raw.len+ChHdrSize])

	if crc != U2(raw.buf[4:6]) {
		log.Debugf("ch checksum error: frame:0x%X calculate:0x%X, len:%d\n", U2(raw.buf[4:6]), crc, raw.len)
		return -1
	}

	return parseData(raw)
}

// parseData parses the data from the rawType
func parseData(raw *rawType) int {
	ofs := 0
	i := 0
	p := raw.buf[ChHdrSize:]

	raw.itemCode = [8]uint8{}
	raw.nItemCode = 0

	for ofs < raw.len {
		switch p[ofs] {
		case kItemID:
			raw.nIMU = 1
			raw.itemCode[raw.nItemCode] = kItemID
			raw.imu[0].ID = uint32(p[ofs+1])
			ofs += 2
		case kItemAccRaw:
			raw.nIMU = 1
			raw.itemCode[raw.nItemCode] = kItemAccRaw
			raw.imu[0].Acc[0] = float32(I2(p[ofs+1:])) / 1000
			raw.imu[0].Acc[1] = float32(I2(p[ofs+3:])) / 1000
			raw.imu[0].Acc[2] = float32(I2(p[ofs+5:])) / 1000
			ofs += 7
		case kItemGyrRaw:
			raw.nIMU = 1
			raw.itemCode[raw.nItemCode] = kItemGyrRaw
			raw.imu[0].Gyro[0] = float32(I2(p[ofs+1:])) / 10
			raw.imu[0].Gyro[1] = float32(I2(p[ofs+3:])) / 10
			raw.imu[0].Gyro[2] = float32(I2(p[ofs+5:])) / 10
			ofs += 7
		case kItemMagRaw:
			raw.nIMU = 1
			raw.itemCode[raw.nItemCode] = kItemMagRaw
			raw.imu[0].Mag[0] = float32(I2(p[ofs+1:])) / 10
			raw.imu[0].Mag[1] = float32(I2(p[ofs+3:])) / 10
			raw.imu[0].Mag[2] = float32(I2(p[ofs+5:])) / 10
			ofs += 7
		case kItemRotationEul:
			raw.itemCode[raw.nItemCode] = kItemRotationEul
			raw.imu[0].Euler[0] = float32(I2(p[ofs+1:])) / 100
			raw.imu[0].Euler[1] = float32(I2(p[ofs+3:])) / 100
			raw.imu[0].Euler[2] = float32(I2(p[ofs+5:])) / 10
			ofs += 7
		case kItemRotationQuat:
			raw.nIMU = 1
			raw.itemCode[raw.nItemCode] = kItemRotationQuat
			raw.imu[0].Quat[0] = R4(p[ofs+1:])
			raw.imu[0].Quat[1] = R4(p[ofs+5:])
			raw.imu[0].Quat[2] = R4(p[ofs+9:])
			raw.imu[0].Quat[3] = R4(p[ofs+13:])
			ofs += 17
		case kItemPressure:
			raw.nIMU = 1
			raw.itemCode[raw.nItemCode] = kItemPressure
			raw.imu[0].Pressure = R4(p[ofs+1:])
			ofs += 5
		case KItemIMUSOL:
			raw.nIMU = 1
			raw.itemCode[raw.nItemCode] = KItemIMUSOL
			raw.imu[0].ID = uint32(p[ofs+1])
			raw.imu[0].Pressure = R4(p[ofs+4:])
			raw.imu[0].Timestamp = U4(p[ofs+8:])
			raw.imu[0].Acc[0] = R4(p[ofs+12:])
			raw.imu[0].Acc[1] = R4(p[ofs+16:])
			raw.imu[0].Acc[2] = R4(p[ofs+20:])
			raw.imu[0].Gyro[0] = R4(p[ofs+24:])
			raw.imu[0].Gyro[1] = R4(p[ofs+28:])
			raw.imu[0].Gyro[2] = R4(p[ofs+32:])
			raw.imu[0].Mag[0] = R4(p[ofs+36:])
			raw.imu[0].Mag[1] = R4(p[ofs+40:])
			raw.imu[0].Mag[2] = R4(p[ofs+44:])
			raw.imu[0].Euler[0] = R4(p[ofs+48:])
			raw.imu[0].Euler[1] = R4(p[ofs+52:])
			raw.imu[0].Euler[2] = R4(p[ofs+56:])
			raw.imu[0].Quat[0] = R4(p[ofs+60:])
			raw.imu[0].Quat[1] = R4(p[ofs+64:])
			raw.imu[0].Quat[2] = R4(p[ofs+68:])
			raw.imu[0].Quat[3] = R4(p[ofs+72:])
			ofs += 76
		case KItemGWSOL:
			raw.itemCode[raw.nItemCode] = KItemGWSOL
			raw.gWID = p[ofs+1]
			raw.nIMU = p[ofs+2]
			ofs += 8
			for i = 0; i < int(raw.nIMU); i++ {
				raw.imu[i].ID = uint32(p[ofs+1])
				raw.imu[i].Pressure = R4(p[ofs+4:])
				raw.imu[i].Timestamp = U4(p[ofs+8:])
				raw.imu[i].Acc[0] = R4(p[ofs+12:])
				raw.imu[i].Acc[1] = R4(p[ofs+16:])
				raw.imu[i].Acc[2] = R4(p[ofs+20:])
				raw.imu[i].Gyro[0] = R4(p[ofs+24:])
				raw.imu[i].Gyro[1] = R4(p[ofs+28:])
				raw.imu[i].Gyro[2] = R4(p[ofs+32:])
				raw.imu[i].Mag[0] = R4(p[ofs+36:])
				raw.imu[i].Mag[1] = R4(p[ofs+40:])
				raw.imu[i].Mag[2] = R4(p[ofs+44:])
				raw.imu[i].Euler[0] = R4(p[ofs+48:])
				raw.imu[i].Euler[1] = R4(p[ofs+52:])
				raw.imu[i].Euler[2] = R4(p[ofs+56:])
				raw.imu[i].Quat[0] = R4(p[ofs+60:])
				raw.imu[i].Quat[1] = R4(p[ofs+64:])
				raw.imu[i].Quat[2] = R4(p[ofs+68:])
				raw.imu[i].Quat[3] = R4(p[ofs+72:])
				ofs += 76
			}
		default:
			ofs++
		}
	}

	return 1
}

func crc16Update(currect_crc *uint16, src []uint8) {
	crc := *currect_crc
	l := len(src)

	for j := 0; j < l; j++ {
		b := src[j]
		crc ^= uint16(b) << 8
		for i := 0; i < 8; i++ {
			temp := crc << 1
			if (crc & 0x8000) != 0 {
				temp ^= 0x1021
			}
			crc = temp
		}
	}
	*currect_crc = crc
}

func U2(p []uint8) uint16 {
	return (uint16(p[1]) << 8) + uint16(p[0])
}

func U4(p []uint8) uint32 {
	return (uint32(p[3]) << 24) + (uint32(p[2]) << 16) + (uint32(p[1]) << 8) + uint32(p[0])
}

func R4(p []uint8) float32 {
	return math.Float32frombits(U4(p))
}

func I2(p []uint8) int16 {
	return (int16(p[1]) << 8) + int16(p[0])
}
