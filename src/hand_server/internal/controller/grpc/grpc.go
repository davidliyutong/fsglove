package grpc

import (
	"context"
	"errors"
	log "github.com/sirupsen/logrus"
	"hand_apiserver/internal/manager"
	pb2 "hand_apiserver/internal/pb"
	"hand_apiserver/internal/sensor"
	"math"
	"sync"
	"time"
)

type server struct {
	pb2.UnimplementedIMUPacketServiceServer
	manager    manager.Manager
	streamLock sync.Mutex
}

// SetFIFOStatus sets the status of the manager
func (s *server) SetFIFOStatus(ctx context.Context, req *pb2.IMUSetFIFOStatusRequest) (*pb2.IMUStatusResponse, error) {
	log.Infof("SetFIFOStatus: %v", req.Status)
	var err error

	// If the request status is true, start the manager
	if req.Status {
		err = s.manager.Start()
	} else {
		err = s.manager.Stop()
	}

	// If there is an error, return the error message
	if err != nil {
		return &pb2.IMUStatusResponse{
			Status: s.manager.Running(),
			Err:    err.Error(),
		}, nil
	} else {
		return &pb2.IMUStatusResponse{
			Status: s.manager.Running(),
			Err:    "",
		}, nil
	}
}

// GetFIFOStatus returns the status of the manager
func (s *server) GetFIFOStatus(ctx context.Context, req *pb2.Empty) (*pb2.IMUStatusResponse, error) {
	log.Infof("GetFIFOStatus: %v", s.manager.Running())
	return &pb2.IMUStatusResponse{
		Status: s.manager.Running(),
		Err:    "",
	}, nil
}

var packetResponseRawPool = sync.Pool{
	New: func() interface{} {
		b := pb2.IMUPacketResponseRaw{}
		return &b
	},
}

// createPacketArray returns the packet from the manager
func (s *server) createPacket(packet *sensor.IMUDataTypeWrapped) *pb2.IMUPacketResponseRaw {
	if packet == nil {
		return nil
	} else {
		// assemble the packet
		res := &pb2.IMUPacketResponseRaw{
			Id:        packet.ID,
			AccelX:    packet.Acc[0],
			AccelY:    packet.Acc[1],
			AccelZ:    packet.Acc[2],
			GyroX:     packet.Gyro[0],
			GyroY:     packet.Gyro[1],
			GyroZ:     packet.Gyro[2],
			MagX:      packet.Mag[0],
			MagY:      packet.Mag[1],
			MagZ:      packet.Mag[2],
			QuatW:     packet.Quat[0],
			QuatX:     packet.Quat[1],
			QuatY:     packet.Quat[2],
			QuatZ:     packet.Quat[3],
			Roll:      packet.Euler[0],
			Pitch:     packet.Euler[1],
			Yaw:       packet.Euler[2],
			Pressure:  packet.Pressure,
			SysTicks:  packet.SysTicks,
			Timestamp: packet.Timestamp,
			Seq:       packet.Seq,
			Valid:     true,
		}
		return res
	}
}

// GetPacketArray returns the packet array from the manager
func (s *server) GetPacketArray(ctx context.Context, req *pb2.IMUPacketRequest) (*pb2.IMUPacketArrayResponse, error) {
	if !s.manager.Running() {
		return nil, errors.New("client repo is not running")
	}
	if s.manager.Faulted() {
		return nil, errors.New("client repo is faulted")
	}

	_, packets, err := s.manager.Read(-1)
	if err != nil {
		return nil, err
	}
	resp := &pb2.IMUPacketArrayResponse{
		Packets:  make(map[string]*pb2.IMUPacketResponseRaw, len(packets)),
		SysTicks: 0,
		Valid:    true,
	}

	minSysTicks := int64(math.MaxInt64)
	for id, packet := range packets[0] {
		resp.Packets[id] = s.createPacket(packet)
		if packet.SysTicks < minSysTicks {
			minSysTicks = packet.SysTicks
		}
	}
	resp.SysTicks = minSysTicks
	return resp, nil
}

// GetPacketArrayStream returns the packet array stream from the manager
func (s *server) GetPacketArrayStream(req *pb2.IMUPacketRequest, srv pb2.IMUPacketService_GetPacketArrayStreamServer) error {
	s.streamLock.Lock()
	defer s.streamLock.Unlock()
	var lastCursor = int64(-1)

	resp := &pb2.IMUPacketArrayStreamResponse{
		Packets: nil,
		Valid:   false,
	}

	lastSuccess := time.Now()
	for {
		if !s.manager.Running() {
			return errors.New("client repo is not running")
		}
		if s.manager.Faulted() {
			return errors.New("client repo is faulted")
		}
		cursor, packets, err := s.manager.Read(lastCursor)
		if err != nil {
			if time.Now().Sub(lastSuccess) > time.Second {
				return err
			}
			time.Sleep(time.Millisecond * 10)
			continue
		}
		lastSuccess = time.Now()

		lastCursor = cursor

		resp.Packets = make([]*pb2.IMUPacketResponseSynced, len(packets), len(packets))
		resp.Valid = true

		for idx, packet := range packets {
			resp.Packets[idx] = &pb2.IMUPacketResponseSynced{Packets: make(map[string]*pb2.IMUPacketResponseRaw)}
			minSysTicks := int64(math.MaxInt64)
			for deviceID, item := range packet {
				resp.Packets[idx].Packets[deviceID] = s.createPacket(item)
				if item.SysTicks < minSysTicks {
					minSysTicks = item.SysTicks
				}
				resp.Packets[idx].SysTicks = minSysTicks
			}
		}
		err = srv.Send(resp)
		for _, packet := range resp.Packets {
			for _, p := range packet.Packets {
				packetResponseRawPool.Put(p)
			}
		}
		if err != nil {
			return err
		}
	}
}

func (s *server) ListDev(ctx context.Context, req *pb2.Empty) (*pb2.IMUInfoResponse, error) {
	res := pb2.IMUInfoResponse{Ids: nil}
	ids, err := s.manager.ListDev()
	res.Ids = ids
	return &res, err
}

var _ pb2.IMUPacketServiceServer = &server{}

func NewGRPCServer(manager manager.Manager) pb2.IMUPacketServiceServer {
	return &server{
		manager: manager,
	}
}
