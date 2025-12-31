package server

import (
	"fmt"
	log "github.com/sirupsen/logrus"
	"github.com/spf13/cobra"
	"google.golang.org/grpc"
	"hand_apiserver/internal/config"
	grpc2 "hand_apiserver/internal/controller/grpc"
	managerImpl "hand_apiserver/internal/manager/hi229"
	"hand_apiserver/internal/pb"
	"hand_apiserver/pkg/version"
	"net"
	"os"
	"strconv"
	"sync"
	"strings"
)

type mainApp struct {
	name string
	cmd  *cobra.Command
	args []string
	opt  *config.RFMoCapOpt
}

func (a *mainApp) ProbeSensor() error {
	m := managerImpl.NewManager(a.opt)
	log.Infoln("Probing IMU devices...")
	res, err := m.ProbeDev()
	if err != nil {
		log.Errorln(err)
		return err
	} else {
		log.Infof("Found %d Valid IMU devices: \n", len(res))
		for _, v := range res {
			fmt.Printf("- %s\n", strings.TrimSpace(v))
		}
	}
	return nil
}

func (a *mainApp) GetOpt() *config.RFMoCapOpt {
	return a.opt
}

func (a *mainApp) SetOpt(opt *config.RFMoCapOpt) { a.opt = opt }

var app MainApp = nil

func (a *mainApp) Run() {
	// init database
	var once sync.Once
	once.Do(func() {
		app = a
	})

	log.Infoln("version:", version.GitVersion)
	log.Infoln("grpc.port:", a.opt.GRPC.Port)
	log.Infoln("grpc.interface:", a.opt.GRPC.Interface)
	log.Infoln("api.port:", a.opt.API.Port)
	log.Infoln("api.interface:", a.opt.API.Interface)
	log.Infoln("debug:", a.opt.Debug)
	log.Infoln("imu.device:", a.opt.IMU)

	// start manager
	m := managerImpl.NewManager(a.opt)
	go managerImpl.Daemon(m)

	// install and start api server

	// install and restart grpc server
	s := grpc.NewServer()
	grpcServer := grpc2.NewGRPCServer(m)
	pb.RegisterIMUPacketServiceServer(s, grpcServer)
	listener, err := net.Listen("tcp", a.opt.GRPC.Interface+":"+strconv.Itoa(a.opt.GRPC.Port))
	if err != nil {
		log.Errorln("net listen err ", err)
		return
	}
	log.Info("start gRPC listen on ", a.opt.GRPC.Interface+":"+strconv.Itoa(a.opt.GRPC.Port))
	if err := s.Serve(listener); err != nil {
		log.Errorln("failed to serve...", err)
		return
	}

	// wait for exit
	select {}
}

func (a *mainApp) PrepareRun() MainApp {
	desc := config.NewRFMoCapDesc()
	err := desc.Parse(a.cmd)
	if err != nil {
		log.Errorln(err)
		os.Exit(1)
		return nil
	}
	desc.PostParse()
	a.opt = &desc.Opt
	a.name = config.DefaultAppName

	if a.opt.Debug {
		log.SetLevel(log.DebugLevel)
	}

	return a
}

type MainApp interface {
	Run()
	PrepareRun() MainApp
	GetOpt() *config.RFMoCapOpt
	SetOpt(*config.RFMoCapOpt)
	ProbeSensor() error
}

func NewMainApp(cmd *cobra.Command, args []string) MainApp {
	return &mainApp{
		cmd:  cmd,
		args: args,
	}
}
