package config

import (
	"bufio"
	"errors"
	"fmt"
	log "github.com/sirupsen/logrus"
	"github.com/spf13/cobra"
	"github.com/spf13/viper"
	"gopkg.in/yaml.v3"
	"hand_apiserver/internal/utils"
	"os"
	"path"
	"strings"
)

const DefaultAppName = "rfmocap"
const DefaultConfigName = "config"
const DefaultGRPCInterface = "0.0.0.0"
const DefaultGRPCPort = 18890
const DefaultAPIInterface = "0.0.0.0"
const DefaultAPIPort = 18889
const DefaultIMUID = "imu_0"

var userHomeDir, _ = os.UserHomeDir()
var DefaultConfig = path.Join(userHomeDir, ".config/"+DefaultAppName+"/"+DefaultConfigName+".yaml")
var DefaultConfigSearchPath0 = path.Join(userHomeDir, ".config", DefaultAppName)

const DefaultConfigSearchPath1 = "/etc/" + DefaultAppName
const DefaultConfigSearchPath2 = "./"
const DefaultConfigSearchPath3 = "/config"

type GRPCOpt struct {
	Port      int    `yaml:"port"`
	Interface string `yaml:"interface"`
}

type APIOpt struct {
	Port      int    `yaml:"port"`
	Interface string `yaml:"interface"`
}

type IMUOpt struct {
	ID   string `yaml:"id"`
	Name string `yaml:"name"`
	Baud int    `yaml:"baud"`
}

type RFMoCapOpt struct {
	GRPC  GRPCOpt  `yaml:"grpc"`
	API   APIOpt   `yaml:"api"`
	IMU   []IMUOpt `yaml:"imu"`
	Debug bool     `yaml:"debug"`
}

type RFMoCapDesc struct {
	Opt   RFMoCapOpt
	Viper *viper.Viper
}

func NewRFMoCapDesc() RFMoCapDesc {
	return RFMoCapDesc{
		Opt:   NewRFMoCapOpt(),
		Viper: nil,
	}
}

func NewRFMoCapOpt() RFMoCapOpt {
	return RFMoCapOpt{
		GRPC: GRPCOpt{
			Port:      DefaultGRPCPort,
			Interface: DefaultGRPCInterface,
		},
		API: APIOpt{
			Port:      DefaultAPIPort,
			Interface: DefaultAPIInterface,
		},
		IMU: []IMUOpt{
			{
				ID: DefaultIMUID,
			},
		},
		Debug: false,
	}
}

func (o *RFMoCapDesc) Parse(cmd *cobra.Command) error {
	vipCfg := viper.New()
	vipCfg.SetDefault("grpc.port", DefaultGRPCPort)
	vipCfg.SetDefault("grpc.interface", DefaultGRPCInterface)
	vipCfg.SetDefault("api.port", DefaultAPIPort)
	vipCfg.SetDefault("api.interface", DefaultAPIInterface)
	vipCfg.SetDefault("debug", false)

	if configFileCmd, err := cmd.Flags().GetString("config"); err == nil && configFileCmd != "" {
		vipCfg.SetConfigFile(configFileCmd)
	} else {
		configFileEnv := os.Getenv("RFMOCAP_CONFIG")
		if configFileEnv != "" {
			vipCfg.SetConfigFile(configFileEnv)
		} else {
			vipCfg.SetConfigName(DefaultConfigName)
			vipCfg.SetConfigType("yaml")
			vipCfg.AddConfigPath(DefaultConfigSearchPath0)
			vipCfg.AddConfigPath(DefaultConfigSearchPath1)
			vipCfg.AddConfigPath(DefaultConfigSearchPath2)
			vipCfg.AddConfigPath(DefaultConfigSearchPath3)
		}
	}
	vipCfg.WatchConfig()

	vipCfg.SetEnvPrefix(DefaultAppName)
	vipCfg.SetEnvKeyReplacer(strings.NewReplacer(".", "_"))
	vipCfg.AutomaticEnv()

	_ = vipCfg.BindPFlag("api.port", cmd.Flags().Lookup("port"))
	_ = vipCfg.BindPFlag("api.interface", cmd.Flags().Lookup("interface"))
	_ = vipCfg.BindPFlag("debug", cmd.Flags().Lookup("debug"))

	// If a config file is found, read it in.
	if err := vipCfg.ReadInConfig(); err == nil {
		log.Debugln("using config file:", vipCfg.ConfigFileUsed())
	} else {
		log.Warnln(err)
	}

	if err := vipCfg.Unmarshal(&o.Opt); err != nil {
		log.Fatalln("failed to unmarshal config")
		os.Exit(1)
	}

	o.Viper = vipCfg
	return nil
}

func (o *RFMoCapDesc) PostParse() {
	if o.Opt.Debug {
		log.SetLevel(log.DebugLevel)
	} else {
		log.SetLevel(log.InfoLevel)
	}
}

func (o *RFMoCapDesc) SaveConfig() error {
	if o.Viper == nil {
		return errors.New("viper is nil")
	}
	f, err := os.OpenFile(o.Viper.ConfigFileUsed(), os.O_CREATE|os.O_WRONLY|os.O_TRUNC, 0644)
	defer func() { _ = f.Close() }()
	if err != nil {
		return err
	}
	w := bufio.NewWriter(f)
	s, _ := yaml.Marshal(o.Opt)
	_, err = w.Write(s)
	if err != nil {
		return err
	}
	_ = w.Flush()
	return nil
}

// InitCfg initConfig prepares config for the application
func InitCfg(cmd *cobra.Command, _ []string) error {
	printFlag, _ := cmd.Flags().GetBool("print")
	outputPath, _ := cmd.Flags().GetString("output")
	overwriteFlag, _ := cmd.Flags().GetBool("yes")

	desc := NewRFMoCapDesc()
	err := desc.Parse(cmd)
	if err != nil {
		log.Errorln(err)
		return err
	}

	if printFlag {
		configBuffer, _ := yaml.Marshal(desc.Opt)
		fmt.Println(string(configBuffer))
	} else {
		utils.DumpOption(desc.Opt, outputPath, overwriteFlag)
	}
	return nil
}
