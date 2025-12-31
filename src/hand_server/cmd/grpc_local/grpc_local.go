package main

import (
	"bufio"
	"context"
	"encoding/json"
	"fmt"
	ui "github.com/gizak/termui/v3"
	"github.com/gizak/termui/v3/widgets"
	log "github.com/sirupsen/logrus"
	"github.com/spf13/cobra"
	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"
	pb2 "hand_apiserver/internal/pb"
	"os"
	"time"
)

var defaultTableValue = [][]string{{"ID", "Euler", "Timestamp"}}

func getTable() *widgets.Table {
	table := widgets.NewTable()
	table.Rows = defaultTableValue
	table.ColumnWidths = []int{10, 30, 20}
	table.TextStyle = ui.NewStyle(ui.ColorWhite)
	table.TextAlignment = ui.AlignRight
	table.SetRect(0, 0, 60, 36)
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

func updateValue(address string, table *widgets.Table) {
	conn, err := grpc.Dial(address, grpc.WithTransportCredentials(insecure.NewCredentials()))
	if err != nil {
		log.Fatalf("did not connect: %v", err)
	}
	defer conn.Close()
	c := pb2.NewIMUPacketServiceClient(conn)
	res, err := c.GetPacketArray(context.Background(), &pb2.IMUPacketRequest{
		Timestamp: uint64(time.Now().UnixNano()),
	})
	if err != nil {
		log.Fatalf("could not get packet array: %v", err)
	}

	tableRowMap := make(map[string]int)
	for _, packet := range res.Packets {
		table.Rows = append(table.Rows, []string{"", "", ""})
		tableRowMap[packet.Id] = len(table.Rows) - 1
	}

	s, err := c.GetPacketArrayStream(context.Background(), &pb2.IMUPacketRequest{
		Timestamp: uint64(time.Now().UnixNano()),
	})
	if err != nil {
		log.Fatalf("could not get packet array stream: %v", err)
	}

	file, err := os.Create(fmt.Sprintf("%v.jsonl", time.Now().Format("2006-01-02T15-04-05")))
	if err != nil {
		log.Fatalf("could not create file: %v", err)
	}
	defer file.Close()
	writer := bufio.NewWriter(file)

	var idx = 0

	var fps = 100
	var startTime = time.Now().UnixMilli()
	var cnt = 0
	for {
		now := time.Now().UnixMilli()
		if (now - startTime) < int64(cnt*1000/fps) {
			time.Sleep(time.Millisecond * 5)
			continue
		}
		resp, err := s.Recv()
		if err != nil {
			log.Fatalf("could not receive packet array: %v", err)
		}
		for _, _raw := range resp.Packets[0].Packets {
			table.Rows[tableRowMap[_raw.Id]] = []string{_raw.Id, printArray([3]float32{_raw.Roll, _raw.Pitch, _raw.Yaw}), fmt.Sprintf("%d", _raw.Timestamp)}
		}
		jsonPacket, err := json.Marshal(resp.Packets)
		if err != nil {
			log.Fatalf("Error marshaling packet to JSON: %v", err)
		}

		// Write the JSON packet to the JSONL file
		_, err = writer.WriteString(string(jsonPacket) + "\n")
		if err != nil {
			log.Fatalf("Error writing packet to file: %v", err)
		}

		// Flush the buffer to ensure data is written to the file

		if idx%100 == 0 {
			err = writer.Flush()
			if err != nil {
				log.Fatalf("Error flushing buffer: %v", err)
			}
		}
		idx++
		cnt++
		ui.Render(table)
	}

}

func _main(cmd *cobra.Command, args []string) {
	log.Info("Starting")
	if err := ui.Init(); err != nil {
		log.Fatalf("failed to initialize termui: %v", err)
	}
	defer ui.Close()

	t := getTable()
	address, _ := cmd.Flags().GetString("address")
	go updateValue(address, t)
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
	rootCmd.Flags().String("address", "127.0.0.1:18890", "default dial address")
	rootCmd.Flags().Bool("debug", false, "toggle debug logging")

	err := rootCmd.Execute()
	if err != nil {
		return
	}
}
