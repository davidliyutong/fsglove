package cmd

import (
	"github.com/spf13/cobra"
	"hand_apiserver/internal/config"
	"hand_apiserver/internal/server"
)

var RootCmd = &cobra.Command{
	Use:   "rfmocap",
	Short: "control/data plane of MoCap tracking system",
	Long:  "control/data plane of MoCap tracking system",
}

func ServeCmdRunE(cmd *cobra.Command, args []string) error {
	server.NewMainApp(cmd, args).PrepareRun().Run()
	return nil
}

func ServeCmdFlags(cmd *cobra.Command) {
	cmd.Flags().String("config", "", "default configuration path")
	cmd.Flags().Int64P("port", "p", config.DefaultAPIPort, "port that nameserver listen on")
	cmd.Flags().StringP("interface", "i", config.DefaultAPIInterface, "interface that nameserver listen on, default to 0.0.0.0")
	cmd.Flags().Bool("debug", false, "toggle debug logging")
}

var ServeCmd = &cobra.Command{
	Use: "serve",
	SuggestFor: []string{
		"ru", "ser",
	},
	Short: "serve start the RFMoCap using predefined configs.",
	Long: `serve start the RFMoCap using predefined configs, by the following order:
1. path specified in --config flag
2. path defined RFMOCAP_CONFIG environment variable
3. default location $HOME/.config/rfmocap/config.yaml, /etc/rfmocap/config.yaml, current directory
The parameters in the configuration file will be overwritten by the following order:
1. command line arguments
2. environment variables
`,
	Example: `  rfmocap-app serve --config=/path/to/config`,
	RunE:    ServeCmdRunE,
}

func InitCmdFlags(cmd *cobra.Command) {
	cmd.Flags().Bool("print", false, "print config to stdout")
	cmd.Flags().BoolP("yes", "y", false, "overwrite")
	cmd.Flags().StringP("output", "o", config.DefaultConfig, "specify output directory")
}

var InitCmd = &cobra.Command{
	Use: "init",
	SuggestFor: []string{
		"ini", "in",
	},
	Short: "init create a configuration template",
	Long: `init create a configuration template.
The configuration file can be used to launch the MoCap server.
If --print flag is present, the configuration will be printed to stdout.
If --output / -o flag is present, the configuration will be saved to the path specified
Otherwise init will output configuration file to $HOME/.config/rfmocap/config.yaml
If --yes / -y flag is present, the configuration will be overwrite without confirmation
`,
	Example: `  rfmocap-app init --print
  rfmocap-app init --output /path/to/config.yaml
  rfmocap-app init -o /path/to/config.yaml -y`,
	RunE: config.InitCfg,
}

var ProbeCmd = &cobra.Command{
	Use: "probe",
	SuggestFor: []string{
		"pro", "pr", "prob",
	},
	Short: "probe the compatible devices",
	Long: `probe the compatible devices.
The probe command will scan the Serial/SPI for compatible IMUs and print the result to stdout.
Warning: Only IMUs running at 115200 baud-rate can be detected.
`,
	Example: `  rfmocap-app probe`,
	Run: func(cmd *cobra.Command, args []string) {
		_ = server.NewMainApp(cmd, args).PrepareRun().ProbeSensor()
	},
}

func getRootCmd() *cobra.Command {

	ServeCmdFlags(ServeCmd)
	RootCmd.AddCommand(ServeCmd)

	InitCmdFlags(InitCmd)
	RootCmd.AddCommand(InitCmd)

	RootCmd.AddCommand(ProbeCmd)

	return RootCmd
}

func Execute() {
	rootCmd := getRootCmd()
	if err := rootCmd.Execute(); err != nil {
		panic(err)
	}
}
