package main

import (
	"fmt"
	ui "github.com/gizak/termui/v3"
	"github.com/gizak/termui/v3/widgets"
	log "github.com/sirupsen/logrus"
	"github.com/spf13/cobra"
	"hand_apiserver/internal/config"
	"hand_apiserver/internal/manager/hi229"
	"hand_apiserver/internal/server"
	"time"
)

var defaultTableValue = [][]string{{"ID", "Name", "Euler", "Timestamp"}}

func getTable() *widgets.Table {
	table := widgets.NewTable()
	table.Rows = defaultTableValue
	table.ColumnWidths = []int{10, 20, 30, 20}
	table.TextStyle = ui.NewStyle(ui.ColorWhite)
	table.TextAlignment = ui.AlignRight
	table.SetRect(0, 0, 80, 36)
	return table
}

func printArray(arr [3]float32) string {
	//str := "["
	str := ""
	for i, num := range arr {
		str += fmt.Sprintf("%.1f", num)
		if i != len(arr)-1 {
			str += ", "
		}
	}
	//str += "]"
	return str
}

func updateValue(opt *config.RFMoCapOpt, table *widgets.Table) {

	manager := hi229.NewManager(opt)
	tableRowMap := make(map[string]int)
	tableNameMap := make(map[string]string)

	for _, imu := range opt.IMU {
		table.Rows = append(table.Rows, []string{"", "", ""})
		tableRowMap[imu.ID] = len(table.Rows) - 1
		tableNameMap[imu.ID] = imu.Name
	}
	err := manager.Start()
	if err != nil {
		log.Panicln(err)
	}

	for {
		_, res, err := manager.Read(-1)
		if err != nil {
			log.Warnln(err)
			continue
		}

		for id, data := range res[0] {
			table.Rows[tableRowMap[id]] = []string{id, tableNameMap[id], printArray(data.Euler), fmt.Sprintf("%d", data.Timestamp)}
		}

		ui.Render(table)
		time.Sleep(time.Millisecond * 10)

	}
}

func _main(cmd *cobra.Command, args []string) {
	log.Info("Starting")
	if err := ui.Init(); err != nil {
		log.Fatalf("failed to initialize termui: %v", err)
	}
	defer ui.Close()

	t := getTable()
	opt := server.NewMainApp(cmd, args).PrepareRun().GetOpt()
	go updateValue(opt, t)

	uiEvents := ui.PollEvents()
	for {
		e := <-uiEvents
		switch e.ID {
		case "q", "<C-c>":
			return
		}
	}

}

var rootCmd = &cobra.Command{
	Use:   "serial_playground",
	Short: "serial_playground",
	Long:  "serial_playground",
	Run: func(cmd *cobra.Command, args []string) {
		_main(cmd, args)
	},
}

func main() {
	rootCmd.Flags().String("config", "", "default configuration path")
	rootCmd.Flags().Int64P("port", "p", config.DefaultAPIPort, "port that nameserver listen on")
	rootCmd.Flags().StringP("interface", "i", config.DefaultAPIInterface, "interface that nameserver listen on, default to 0.0.0.0")
	rootCmd.Flags().Bool("debug", false, "toggle debug logging")

	err := rootCmd.Execute()
	if err != nil {
		return
	}
}
